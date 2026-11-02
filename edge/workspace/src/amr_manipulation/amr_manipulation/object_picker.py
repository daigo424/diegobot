import threading
import time

import numpy as np
import rclpy
from amr_interfaces.action import PickObject
from amr_interfaces.srv import FindObjects
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from amr_manipulation.top_down_ik import TopDownIK

# 実行中に止まったと判断するMoveItの結果。これ以外の失敗は計画できなかったものとして扱う。
EXECUTION_ERRORS = {MoveItErrorCodes.CONTROL_FAILED, MoveItErrorCodes.TIMED_OUT, MoveItErrorCodes.PREEMPTED,
                    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE}


class PickFailed(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class ObjectPicker(Node):
    def __init__(self):
        super().__init__('object_picker')
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
        self.velocity_scaling = p('velocity_scaling', 0.5).value
        self.gripper_open = p('gripper_open', 1.4).value
        self.gripper_close = p('gripper_close', -0.1745).value
        self.gripper_effort = p('gripper_effort', 0.0).value
        self.empty_grip_position = p('empty_grip_position', -0.10).value
        self.planning_group = p('planning_group', 'arm_group').value

        group = ReentrantCallbackGroup()
        self.ik = None
        self.joints = {}
        self.busy = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.on_description, latched, callback_group=group)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10, callback_group=group)
        self.find_client = self.create_client(FindObjects, '/object_finder/find', callback_group=group)
        self.move_client = ActionClient(self, MoveGroup, '/move_action', callback_group=group)
        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       '/arm_group_controller/follow_joint_trajectory', callback_group=group)
        self.grip_client = ActionClient(self, GripperCommand, '/grip_group_controller/gripper_cmd',
                                        callback_group=group)
        ActionServer(self, PickObject, '~/pick', self.execute, goal_callback=self.on_goal,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)

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

    def execute(self, goal_handle):
        result = PickObject.Result()
        opened = False
        try:
            if self.ik is None:
                raise PickFailed(PickObject.Result.PLANNING_FAILED, 'robot_description not received yet')
            self.stage(goal_handle, 'looking')
            self.move_to(dict(zip(self.arm_joints, self.look_joints)))
            time.sleep(self.settle_time)

            self.stage(goal_handle, 'finding')
            obj, plan = self.choose(goal_handle.request, result)

            self.stage(goal_handle, 'opening')
            self.grip(self.gripper_open)
            opened = True
            self.stage(goal_handle, 'approaching')
            self.move_to(plan['pregrasp'])
            self.stage(goal_handle, 'grasping')
            self.move_linear(plan['line'], plan['pregrasp_z'], plan['grasp_z'])
            if self.grip(self.gripper_close) <= self.empty_grip_position:
                raise PickFailed(PickObject.Result.GRASP_FAILED, 'gripper closed on nothing')
            self.stage(goal_handle, 'lifting')
            self.move_linear(plan['line'], plan['grasp_z'], plan['grasp_z'] + self.lift_height)
            self.stage(goal_handle, 'retreating')
            self.move_to(dict(zip(self.arm_joints, self.look_joints)))
            result.error_code = PickObject.Result.SUCCESS
            result.message = f'picked {obj.label} ({obj.color})'
            goal_handle.succeed()
        except PickFailed as e:
            result.error_code, result.message = e.code, str(e)
            self.get_logger().warn(f'pick failed: {e}')
            if opened and e.code != PickObject.Result.CANCELED:
                self.recover()
            if e.code == PickObject.Result.CANCELED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        finally:
            self.busy = False
        return result

    def stage(self, goal_handle, name):
        if goal_handle.is_cancel_requested:
            raise PickFailed(PickObject.Result.CANCELED, f'canceled before {name}')
        goal_handle.publish_feedback(PickObject.Feedback(stage=name))
        self.get_logger().info(name)

    def choose(self, request, result):
        # YOLOEは色の語があると物の種類の確信度が上がる一方、色の違う物も拾うため、色は判定結果でも絞る。
        target = f'{request.color} {request.query}'.strip()
        # ラズパイの負荷で画像が1秒以上遅れて届くことがあり、その間object_finderは古いとして断るため、少し待って探し直す。
        for _ in range(self.find_attempts):
            response = self.call(self.find_client, FindObjects.Request(queries=[target]), 15.0)
            if response is not None and response.success:
                break
            time.sleep(0.5)
        if response is None or not response.success:
            raise PickFailed(PickObject.Result.NOT_FOUND, response.message if response else 'object_finder did not answer')
        candidates = [o for o in response.objects
                      if o.position_valid and (not request.color or o.color == request.color)]
        if not candidates:
            raise PickFailed(PickObject.Result.NOT_FOUND, f'no {target} with depth')
        candidates.sort(key=lambda o: o.score, reverse=True)
        for obj in candidates:
            plan = self.plan_grasp(obj)
            if plan is not None:
                result.object = obj
                return obj, plan
        result.object = candidates[0]
        p = candidates[0].position.point
        raise PickFailed(PickObject.Result.OUT_OF_REACH, f'{candidates[0].label} at ({p.x:.2f}, {p.y:.2f}) is out of reach')

    def plan_grasp(self, obj):
        p = obj.position.point
        if obj.position.header.frame_id != self.base_frame or p.x < self.min_reach_x:
            return None
        # 位置はカメラから見えた表面(見下ろしているので上面付近)の点なので、少し下をつかむ。
        grasp_z = max(p.z - self.grasp_depth, self.min_tool_z)
        pregrasp_z = grasp_z + self.pregrasp_height
        line = (p.x, p.y)
        grasp, pregrasp, lifted = (self.solve(line, z) for z in (grasp_z, pregrasp_z, grasp_z + self.lift_height))
        if grasp is None or pregrasp is None or lifted is None:
            return None
        return {'line': line, 'grasp_z': grasp_z, 'pregrasp_z': pregrasp_z, 'pregrasp': pregrasp}

    def solve(self, xy, z, previous=None):
        seeds = [previous] if previous else []
        seeds.append({n: self.joints.get(n, 0.0) for n in self.arm_joints})
        n = len(self.arm_joints)
        seeds += [dict(zip(self.arm_joints, self.ik_seeds[i:i + n])) for i in range(0, len(self.ik_seeds), n)]
        return self.ik.solve([xy[0], xy[1], z], self.grasp_roll, seeds)

    def move_to(self, joints):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.planning_group
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.velocity_scaling
        req.max_acceleration_scaling_factor = self.velocity_scaling
        req.start_state.is_diff = True
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=float(joints[n]), tolerance_above=0.01, tolerance_below=0.01,
                            weight=1.0) for n in self.arm_joints])]
        res = self.send(self.move_client, goal, 60.0)
        code = res.error_code.val if res else MoveItErrorCodes.FAILURE
        if code != MoveItErrorCodes.SUCCESS:
            failed = PickObject.Result.EXECUTION_FAILED if code in EXECUTION_ERRORS else PickObject.Result.PLANNING_FAILED
            raise PickFailed(failed, f'move_group failed (error code {code})')

    def move_linear(self, xy, z_from, z_to):
        # MoveItの直線補間はIKを使い、5関節のこのアームでは解けないため、ここで解いた点をつなぐ。
        steps = max(2, int(np.ceil(abs(z_to - z_from) / 0.01)) + 1)
        points, previous = [], None
        dt = abs(z_to - z_from) / (steps - 1) / self.linear_speed
        for i, z in enumerate(np.linspace(z_from, z_to, steps)):
            q = self.solve(xy, z, previous)
            if q is None:
                raise PickFailed(PickObject.Result.PLANNING_FAILED, f'no IK at z={z:.3f}')
            previous = q
            t = 1.0 + i * dt
            points.append(JointTrajectoryPoint(positions=[float(q[n]) for n in self.arm_joints],
                                               time_from_start=Duration(sec=int(t), nanosec=int(t % 1 * 1e9))))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.arm_joints)
        goal.trajectory.points = points
        res = self.send(self.arm_client, goal, points[-1].time_from_start.sec + 10.0)
        if res is None or res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise PickFailed(PickObject.Result.EXECUTION_FAILED,
                             f'linear move failed: {res.error_string if res else "no result"}')

    def grip(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.gripper_effort)
        res = self.send(self.grip_client, goal, 10.0)
        if res is None:
            raise PickFailed(PickObject.Result.EXECUTION_FAILED, 'gripper did not answer')
        return res.position

    def recover(self):
        try:
            self.grip(self.gripper_open)
            self.move_to(dict(zip(self.arm_joints, self.look_joints)))
        except PickFailed as e:
            self.get_logger().error(f'recovery failed: {e}')

    def call(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        return self.wait(client.call_async(request), timeout)

    def send(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=2.0):
            return None
        handle = self.wait(client.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return None
        res = self.wait(handle.get_result_async(), timeout)
        return res.result if res else None

    @staticmethod
    def wait(future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(timeout) else None


def main():
    rclpy.init()
    node = ObjectPicker()
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
