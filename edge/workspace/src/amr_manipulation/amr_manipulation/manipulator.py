import time

import numpy as np
import rclpy
from amr_interfaces.action import PickObject, PlaceObject
from amr_interfaces.srv import FindObjects
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RclDuration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
import tf2_geometry_msgs  # noqa: F401  PointStampedをBuffer.transformで変換できるよう登録する

from amr_manipulation.ros_util import EXECUTION_ERRORS, find_objects, move_joints, send
from amr_manipulation.top_down_ik import TopDownIK


class ArmFailed(Exception):
    """失敗の種類(kind)は、つかむ・置くそれぞれのerror_codeへ対応させる。"""

    def __init__(self, kind, message):
        super().__init__(message)
        self.kind = kind


PICK_CODES = {'not_found': PickObject.Result.NOT_FOUND, 'out_of_reach': PickObject.Result.OUT_OF_REACH,
              'planning': PickObject.Result.PLANNING_FAILED, 'execution': PickObject.Result.EXECUTION_FAILED,
              'grasp': PickObject.Result.GRASP_FAILED, 'canceled': PickObject.Result.CANCELED}
PLACE_CODES = {'not_holding': PlaceObject.Result.NOT_HOLDING, 'out_of_reach': PlaceObject.Result.OUT_OF_REACH,
               'planning': PlaceObject.Result.PLANNING_FAILED, 'execution': PlaceObject.Result.EXECUTION_FAILED,
               'canceled': PlaceObject.Result.CANCELED}


class Manipulator(Node):
    """カメラで見つけた物をつかみ、持っている物を置く(真下向きに上下する)アーム動作。"""

    def __init__(self):
        super().__init__('manipulator')
        p = self.declare_parameter
        self.base_frame = p('base_frame', 'base_link').value
        self.tip_link = p('tip_link', 'gripper_frame_link').value
        self.arm_joints = p('arm_joints', ['']).value
        self.roll_joint = p('roll_joint', 'wrist_roll').value
        self.look_joints = p('look_joints', [0.0]).value
        self.ik_seeds = p('ik_seeds', [0.0]).value
        self.grasp_roll = p('grasp_roll', 0.0).value
        self.tcp_offset = p('tcp_offset', [0.0, 0.0, 0.0]).value
        self.min_reach_x = p('min_reach_x', 0.10).value
        self.pregrasp_height = p('pregrasp_height', 0.08).value
        self.grasp_depth = p('grasp_depth', 0.015).value
        self.min_tool_z = p('min_tool_z', 0.012).value
        self.lift_height = p('lift_height', 0.10).value
        self.linear_speed = p('linear_speed', 0.03).value
        self.settle_time = p('settle_time', 0.8).value
        self.find_attempts = p('find_attempts', 4).value
        self.find_min_score = p('find_min_score', 0.15).value
        self.velocity_scaling = p('velocity_scaling', 0.5).value
        self.gripper_open = p('gripper_open', 1.4).value
        self.gripper_close = p('gripper_close', -0.1745).value
        self.gripper_effort = p('gripper_effort', 0.0).value
        self.empty_grip_position = p('empty_grip_position', -0.10).value
        self.planning_group = p('planning_group', 'arm_group').value
        self.floor_z = p('floor_z', -0.00945).value
        self.default_place = p('default_place', [0.18, 0.0]).value
        self.place_clearance = p('place_clearance', 0.005).value

        group = ReentrantCallbackGroup()
        self.ik = None
        self.joints = {}
        self.busy = False
        # つかんだ物の底から道具点までの高さ。置くときに、物の底が面に着く高さまで下ろすために使う。
        self.held_tool_height = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.on_description, latched, callback_group=group)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10, callback_group=group)
        self.find_client = self.create_client(FindObjects, '/object_finder/find', callback_group=group)
        self.move_client = ActionClient(self, MoveGroup, '/move_action', callback_group=group)
        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       '/arm_group_controller/follow_joint_trajectory', callback_group=group)
        self.grip_client = ActionClient(self, GripperCommand, '/grip_group_controller/gripper_cmd',
                                        callback_group=group)
        cancel = lambda _: CancelResponse.ACCEPT  # noqa: E731
        ActionServer(self, PickObject, '~/pick', self.execute_pick, goal_callback=self.on_goal,
                     cancel_callback=cancel, callback_group=group)
        ActionServer(self, PlaceObject, '~/place', self.execute_place, goal_callback=self.on_goal,
                     cancel_callback=cancel, callback_group=group)

    def on_description(self, msg):
        self.ik = TopDownIK(msg.data, self.base_frame, self.tip_link, self.roll_joint, tcp_offset=self.tcp_offset)

    def on_joint_states(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def on_goal(self, _):
        # アームは1本なので、実行中の依頼があれば新しい依頼は断る。
        if self.busy:
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    # --- つかむ ---

    def execute_pick(self, goal_handle):
        result = PickObject.Result()
        opened = False
        try:
            self.require_ik()
            # 近づいた直後はカメラが物へ向いているため、まず姿勢を変えずに探す。姿勢を変えると写り方と明るさが変わり、
            # 見えていた物を見落とすことがある。届く物が無ければ見下ろす姿勢で探し直す。
            self.stage(goal_handle, PickObject, 'finding')
            try:
                obj, plan = self.choose(goal_handle.request, result)
            except ArmFailed as e:
                self.get_logger().info(f'not in reach from the current pose ({e}), looking down')
                self.stage(goal_handle, PickObject, 'looking')
                self.move_to(dict(zip(self.arm_joints, self.look_joints)))
                time.sleep(self.settle_time)
                self.stage(goal_handle, PickObject, 'finding')
                obj, plan = self.choose(goal_handle.request, result)

            self.stage(goal_handle, PickObject, 'opening')
            self.grip(self.gripper_open)
            opened = True
            self.stage(goal_handle, PickObject, 'approaching')
            self.move_to(plan['above'])
            self.stage(goal_handle, PickObject, 'grasping')
            self.move_linear(plan['line'], plan['above_z'], plan['z'])
            if self.grip(self.gripper_close) <= self.empty_grip_position:
                raise ArmFailed('grasp', 'gripper closed on nothing')
            # 物は床に置かれている前提で、底(床)から道具点までの高さを覚えておく。
            self.held_tool_height = plan['z'] - self.floor_z
            self.stage(goal_handle, PickObject, 'lifting')
            self.move_linear(plan['line'], plan['z'], plan['z'] + self.lift_height)
            self.stage(goal_handle, PickObject, 'retreating')
            self.move_to(dict(zip(self.arm_joints, self.look_joints)))
            result.error_code = PickObject.Result.SUCCESS
            result.message = f'picked {obj.label} ({obj.color})'
            goal_handle.succeed()
        except ArmFailed as e:
            self.finish_failed(goal_handle, result, PICK_CODES, e, 'pick', recover=opened)
        finally:
            self.busy = False
        return result

    def choose(self, request, result):
        candidates, message = find_objects(self.find_client, request.queries, request.color, self.find_attempts,
                                           self.find_min_score)
        if not candidates:
            raise ArmFailed('not_found', message)
        for obj in candidates:
            p = obj.position.point
            if obj.position.header.frame_id != self.base_frame:
                continue
            # 位置はカメラから見えた表面(見下ろしているので上面付近)の点なので、少し下をつかむ。
            plan = self.plan_vertical((p.x, p.y), max(p.z - self.grasp_depth, self.min_tool_z))
            if plan is not None:
                result.object = obj
                return obj, plan
        result.object = candidates[0]
        p = candidates[0].position.point
        raise ArmFailed('out_of_reach', f'{candidates[0].label} at ({p.x:.2f}, {p.y:.2f}) is out of reach')

    # --- 置く ---

    def execute_place(self, goal_handle):
        req, result = goal_handle.request, PlaceObject.Result()
        try:
            self.require_ik()
            # 閉じきっている(空振り)か開いている(何も持っていない)なら、持っていないとみなす。
            gripper = self.joints.get('gripper', self.gripper_close)
            if not self.empty_grip_position < gripper < self.gripper_open - 0.2:
                raise ArmFailed('not_holding', 'the gripper is not holding anything')
            if req.release_here:
                self.stage(goal_handle, PlaceObject, 'releasing')
                self.grip(self.gripper_open)
            else:
                line, surface = self.place_target(req.position)
                held = self.held_tool_height if self.held_tool_height is not None else self.min_tool_z - self.floor_z
                plan = self.plan_vertical(line, max(surface + held + self.place_clearance, self.min_tool_z))
                if plan is None:
                    raise ArmFailed('out_of_reach', f'({line[0]:.2f}, {line[1]:.2f}) is out of reach')
                self.stage(goal_handle, PlaceObject, 'approaching')
                self.move_to(plan['above'])
                self.stage(goal_handle, PlaceObject, 'lowering')
                self.move_linear(line, plan['above_z'], plan['z'])
                self.stage(goal_handle, PlaceObject, 'releasing')
                self.grip(self.gripper_open)
                self.stage(goal_handle, PlaceObject, 'lifting')
                self.move_linear(line, plan['z'], plan['z'] + self.lift_height)
                self.stage(goal_handle, PlaceObject, 'retreating')
                self.move_to(dict(zip(self.arm_joints, self.look_joints)))
            self.held_tool_height = None
            result.error_code, result.message = PlaceObject.Result.SUCCESS, 'released'
            goal_handle.succeed()
        except ArmFailed as e:
            self.finish_failed(goal_handle, result, PLACE_CODES, e, 'place', recover=False)
        finally:
            self.busy = False
        return result

    def place_target(self, position):
        """置く位置を(base_linkでの水平位置, 面の高さ)で返す。frame_idが空なら車体の前の床。"""
        if not position.header.frame_id:
            return tuple(self.default_place), self.floor_z
        try:
            p = self.tf_buffer.transform(position, self.base_frame, timeout=RclDuration(seconds=0.5)).point
        except TransformException as e:
            raise ArmFailed('planning', f'cannot transform the place position: {e}')
        return (p.x, p.y), p.z

    # --- 共通 ---

    def require_ik(self):
        if self.ik is None:
            raise ArmFailed('planning', 'robot_description not received yet')

    def stage(self, goal_handle, action, name):
        if goal_handle.is_cancel_requested:
            raise ArmFailed('canceled', f'canceled before {name}')
        goal_handle.publish_feedback(action.Feedback(stage=name))
        self.get_logger().info(name)

    def finish_failed(self, goal_handle, result, codes, error, what, recover):
        result.error_code, result.message = codes[error.kind], str(error)
        self.get_logger().warn(f'{what} failed: {error}')
        if error.kind == 'canceled':
            goal_handle.canceled()
            return
        if recover:
            self.recover()
        goal_handle.abort()

    def plan_vertical(self, line, z):
        """lineの真上pregrasp_heightから高さzまで真下向きに上下できるか確かめ、上の姿勢を返す。"""
        if line[0] < self.min_reach_x:
            return None
        above_z = z + self.pregrasp_height
        down, above, lifted = (self.solve(line, h) for h in (z, above_z, z + self.lift_height))
        if down is None or above is None or lifted is None:
            return None
        return {'line': line, 'z': z, 'above_z': above_z, 'above': above}

    def solve(self, xy, z, previous=None):
        seeds = [previous] if previous else []
        seeds.append({n: self.joints.get(n, 0.0) for n in self.arm_joints})
        n = len(self.arm_joints)
        seeds += [dict(zip(self.arm_joints, self.ik_seeds[i:i + n])) for i in range(0, len(self.ik_seeds), n)]
        return self.ik.solve([xy[0], xy[1], z], self.grasp_roll, seeds)

    def move_to(self, joints):
        code = move_joints(self.move_client, self.planning_group, self.arm_joints, joints, self.velocity_scaling)
        if code != MoveItErrorCodes.SUCCESS:
            raise ArmFailed('execution' if code in EXECUTION_ERRORS else 'planning',
                            f'move_group failed (error code {code})')

    def move_linear(self, xy, z_from, z_to):
        # MoveItの直線補間はIKを使い、5関節のこのアームでは解けないため、ここで解いた点をつなぐ。
        steps = max(2, int(np.ceil(abs(z_to - z_from) / 0.01)) + 1)
        points, previous = [], None
        dt = abs(z_to - z_from) / (steps - 1) / self.linear_speed
        for i, z in enumerate(np.linspace(z_from, z_to, steps)):
            q = self.solve(xy, z, previous)
            if q is None:
                raise ArmFailed('planning', f'no IK at z={z:.3f}')
            previous = q
            t = 1.0 + i * dt
            points.append(JointTrajectoryPoint(positions=[float(q[n]) for n in self.arm_joints],
                                               time_from_start=Duration(sec=int(t), nanosec=int(t % 1 * 1e9))))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.arm_joints)
        goal.trajectory.points = points
        res = send(self.arm_client, goal, points[-1].time_from_start.sec + 10.0)
        if res is None or res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise ArmFailed('execution', f'linear move failed: {res.error_string if res else "no result"}')

    def grip(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.gripper_effort)
        res = send(self.grip_client, goal, 10.0)
        if res is None:
            raise ArmFailed('execution', 'gripper did not answer')
        return res.position

    def recover(self):
        try:
            self.grip(self.gripper_open)
            self.move_to(dict(zip(self.arm_joints, self.look_joints)))
        except ArmFailed as e:
            self.get_logger().error(f'recovery failed: {e}')


def main():
    rclpy.init()
    node = Manipulator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
