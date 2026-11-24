import copy
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
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
import tf2_geometry_msgs  # noqa: F401  PointStamped・PoseStampedをBuffer.transformで変換できるよう登録する

from amr_manipulation.ros_util import EXECUTION_ERRORS, call, find_objects, move_joints, send
from amr_manipulation.top_down_ik import SideGraspIK, TopDownIK


def yaw_of(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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
        self.prior_gate = p('prior_gate', 0.06).value
        self.grip_width = p('grip_width', 0.05).value
        self.grip_margin = p('grip_margin', 0.005).value
        self.camera_link = p('camera_link', 'camera_camera_link_0').value
        self.head_match_gate = p('head_match_gate', 0.05).value
        self.head_view_joints = p('head_view_joints', [0.0]).value
        self.side_ratio = p('side_ratio', 1.2).value
        self.side_tcp_offset = p('side_tcp_offset', [0.0, 0.0, 0.0]).value
        self.side_up_axis = p('side_up_axis', [0.0, 1.0, 0.0]).value
        self.side_pitches = np.radians(p('side_pitches', [0.0]).value)
        self.side_seeds = p('side_seeds', [0.0]).value
        self.side_pregrasp = p('side_pregrasp', 0.06).value
        self.side_min_z = p('side_min_z', 0.03).value
        self.side_height_ratio = p('side_height_ratio', 0.7).value
        self.side_gripper_open = p('side_gripper_open', 0.7).value
        self.side_lift = p('side_lift', 0.04).value
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
        self.prior_yaw = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.on_description, latched, callback_group=group)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10, callback_group=group)
        self.find_client = self.create_client(FindObjects, '/object_finder/find', callback_group=group)
        self.head_find_client = self.create_client(FindObjects, p('head_find_service', '/head_object_finder/find').value,
                                                   callback_group=group)
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
        self.side_ik = SideGraspIK(msg.data, self.base_frame, self.tip_link, tcp_offset=self.side_tcp_offset,
                                   up_axis=self.side_up_axis)

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
            self.grip(self.side_gripper_open if plan.get('side') else self.gripper_open)
            opened = True
            self.stage(goal_handle, PickObject, 'approaching')
            self.move_to(plan['above'])
            self.stage(goal_handle, PickObject, 'grasping')
            if plan.get('side'):
                self.move_side_linear(plan['start'], plan['end'], plan['pitch'], plan['direction'])
            else:
                self.move_linear(plan['line'], plan['above_z'], plan['z'], plan.get('roll'))
            if self.grip(self.gripper_close) <= self.empty_grip_position:
                raise ArmFailed('grasp', 'gripper closed on nothing')
            # 物は床に置かれている前提で、底(床)から道具点までの高さを覚えておく。
            self.held_tool_height = plan['z'] - self.floor_z
            self.stage(goal_handle, PickObject, 'lifting')
            if plan.get('side'):
                end = np.asarray(plan['end'])
                self.move_side_linear(end, end + [0.0, 0.0, self.side_lift], plan['pitch'], plan['direction'])
            else:
                self.move_linear(plan['line'], plan['z'], plan['z'] + self.lift_height, plan.get('roll'))
            self.stage(goal_handle, PickObject, 'retreating')
            self.move_to(dict(zip(self.arm_joints, self.look_joints)))
            result.error_code = PickObject.Result.SUCCESS
            # 名前で指定されたときは、色(画素の多数決)を見た目の説明と取り違えないよう言わない。
            result.message = f'picked {obj.label}' if goal_handle.request.target_name else f'picked {obj.label} ({obj.color})'
            goal_handle.succeed()
        except ArmFailed as e:
            self.finish_failed(goal_handle, result, PICK_CODES, e, 'pick', recover=opened)
        finally:
            self.busy = False
        return result

    def choose(self, request, result):
        # 名前で指定されたときは色で絞らない(見分けた物の色が、色の判定と合うとは限らないため)。
        color = '' if request.target_name else request.color
        candidates, message = find_objects(self.find_client, request.queries, color, self.find_attempts,
                                           self.find_min_score)
        prior = self.prior_point(request.target_object)
        if prior is not None and candidates:
            return self.choose_prior(prior, request.target_object, candidates, result)
        if prior is not None and self.is_tall(request.target_object):
            # 横からつかむ物は手首カメラの見下ろす範囲より奥にあり写らないため、近づく動作が測った位置をそのまま使う。
            self.get_logger().warn('the wrist camera does not see the target; using the given position as is')
            return self.choose_prior(prior, request.target_object, None, result)
        if not candidates:
            raise ArmFailed('not_found', message)
        if not candidates[0].width:
            # 手首カメラは近いと物の深度が取れず、床へ投影した位置が奥へずれる。頭部カメラで同じ物が測れればそれを使う。
            head = self.head_counterpart(request.queries, candidates[0])
            if head is not None:
                return self.choose_prior(self.prior_point(head), head, None, result)
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

    def prior_point(self, obj):
        """渡された対象の中心(OBBがあればその中心、無ければ見えた面の点)を、今のbase_linkでの(x, y, z)で返す。"""
        if not obj.position.header.frame_id:
            return None
        # odomは床に固定されているため、測った時刻ではなく今の変換で直せばよい(測った時刻のTFは残っていないことがある)。
        source = obj.obb_pose if obj.obb_valid else obj.position
        source.header.stamp = Time().to_msg()
        try:
            moved = self.tf_buffer.transform(source, self.base_frame, timeout=RclDuration(seconds=0.5))
        except TransformException as e:
            raise ArmFailed('planning', f'cannot transform the target: {e}')
        p = moved.pose.position if obj.obb_valid else moved.point
        self.prior_yaw = yaw_of(moved.pose.orientation) if obj.obb_valid else None
        return p.x, p.y, p.z

    def head_counterpart(self, queries, wrist_obj):
        """手首カメラの検出と同じ物を頭部カメラで探し、OBB付きの検出を返す。無ければNone。
        深度が無くても手首カメラから物への視線の向きは正しいため、頭部の物の中心がその視線の近くにあるかで照らし合わせる。"""
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.camera_link, Time(),
                                                timeout=RclDuration(seconds=0.5)).transform.translation
        except TransformException as e:
            self.get_logger().warn(f'cannot locate the wrist camera: {e}')
            return None
        origin = np.array([t.x, t.y, t.z])
        p = wrist_obj.position.point
        ray = np.array([p.x, p.y, p.z]) - origin
        ray /= np.linalg.norm(ray)
        for attempt in range(2):
            if attempt:
                # アームが頭部カメラの視界をふさいでいることがあるため、よけてからもう一度探す。
                self.get_logger().info('the head camera does not see it; moving the arm aside')
                self.move_to(dict(zip(self.arm_joints, self.head_view_joints)))
                time.sleep(self.settle_time)
            response = call(self.head_find_client, FindObjects.Request(queries=list(queries), min_score=float(
                self.find_min_score), want_obb=True), 15.0)
            objects = [o for o in (response.objects if response is not None and response.success else [])
                       if o.obb_valid and o.position.header.frame_id == self.base_frame]
            best = None
            for obj in objects:
                c = obj.obb_pose.pose.position
                v = np.array([c.x, c.y, c.z]) - origin
                distance = np.linalg.norm(v - (v @ ray) * ray)
                if distance <= self.head_match_gate and (best is None or distance < best[0]):
                    best = (distance, obj)
            if best is not None:
                c = best[1].obb_pose.pose.position
                self.get_logger().info(f'using the head camera: center ({c.x:.3f}, {c.y:.3f}, {c.z:.3f}), '
                                       f'{best[0]:.3f} m from the wrist camera ray')
                return best[1]
        return None

    def choose_prior(self, prior, target, candidates, result):
        """近づく動作が測った対象の中心をつかむ。今の検出は、その近くに物があることの確認にだけ使う
        (手首カメラは近いと物の深度が取れず、床へ投影した位置が奥へずれるため)。"""
        # candidatesがNoneなら、呼び出し側が今の検出と照らし合わせ済み。
        near = candidates is None or [o for o in candidates if o.position.header.frame_id == self.base_frame and np.hypot(
            o.position.point.x - prior[0], o.position.point.y - prior[1]) <= self.prior_gate]
        if not near:
            raise ArmFailed('not_found', f'nothing detected within {self.prior_gate:.2f} m of the target at '
                                         f'({prior[0]:.2f}, {prior[1]:.2f})')
        result.object = self.in_base(target)
        if self.is_tall(target):
            # 背の高い物は重心が中心より上にあり、中心より下を挟むと持ち上げたときに回って落ちるため、上の方を挟む。
            plan = self.plan_side((prior[0], prior[1], self.floor_z + self.side_height_ratio * target.obb_extents.z))
            if plan is None:
                raise ArmFailed('out_of_reach', f'the tall target at ({prior[0]:.2f}, {prior[1]:.2f}) cannot be '
                                                f'grasped from the side')
            self.get_logger().info(f'grasping the given target from the side at ({prior[0]:.3f}, {prior[1]:.3f}, '
                                   f'{plan["z"]:.3f}), {np.degrees(plan["pitch"]):.0f} deg below horizontal')
            return target, plan
        # フィギュアのように端をつかむと外れる物があるため、中心の真上から下ろし、高さは中心(OBBなら高さの半分)にする。
        z = max(prior[2] if target.obb_valid else prior[2] - self.grasp_depth, self.min_tool_z)
        if target.obb_valid and target.obb_extents.y > self.grip_width - 2.0 * self.grip_margin:
            raise ArmFailed('out_of_reach', f'the target is {target.obb_extents.y:.3f} m wide at its narrowest, '
                                            f'wider than the gripper opens')
        roll = self.roll_across(target, (prior[0], prior[1]), z)
        plan = self.plan_vertical((prior[0], prior[1]), z, roll)
        if plan is None:
            raise ArmFailed('out_of_reach', f'the target at ({prior[0]:.2f}, {prior[1]:.2f}) is out of reach')
        self.get_logger().info(f'grasping the given target at ({prior[0]:.3f}, {prior[1]:.3f}, {z:.3f})')
        return target, plan

    def in_base(self, obj):
        """呼び出し側(robot_tools)は結果の物の位置をbase_linkとして距離と向きを出すため、odomのままにしない。"""
        out = copy.deepcopy(obj)
        # 物は床に置かれて動かないため、測った時刻ではなく今の変換で直す。
        out.position.header.stamp = out.obb_pose.header.stamp = Time().to_msg()
        try:
            out.position = self.tf_buffer.transform(out.position, self.base_frame, timeout=RclDuration(seconds=0.5))
            if obj.obb_valid:
                out.obb_pose = self.tf_buffer.transform(out.obb_pose, self.base_frame, timeout=RclDuration(seconds=0.5))
        except TransformException as e:
            self.get_logger().warn(f'cannot transform the target to {self.base_frame}: {e}')
        return out

    def roll_across(self, target, xy, z):
        """指が閉じる向き(道具のx軸)を、物の短辺の向きに合わせる手首のひねり。向きが分からなければ既定のひねり。"""
        if not target.obb_valid or self.prior_yaw is None:
            return self.grasp_roll
        q = self.solve(xy, z)
        if q is None:
            return self.grasp_roll
        x = self.ik.fk(q)[:2, 0]
        # 短辺はOBBのy軸。指は左右どちらから閉じても挟めるため、ひねりの差は±90°以内に収める。
        delta = (self.prior_yaw + np.pi / 2 - np.arctan2(x[1], x[0]) + np.pi / 2) % np.pi - np.pi / 2
        lo, hi = self.ik.lower[self.roll_joint], self.ik.upper[self.roll_joint]
        for roll in (self.grasp_roll + delta, self.grasp_roll + delta - np.pi, self.grasp_roll + delta + np.pi):
            if lo + 0.02 <= roll <= hi - 0.02:
                self.get_logger().info(f'turning the wrist by {np.degrees(roll - self.grasp_roll):.0f} deg '
                                       f'to close across the short side')
                return roll
        return self.grasp_roll

    def is_tall(self, obj):
        """縦に長い物(高さが水平の長辺よりside_ratio倍以上大きい)なら、真上より横からつかむほうが安定する。"""
        e = obj.obb_extents
        return obj.obb_valid and e.z > self.side_ratio * e.x

    def plan_side(self, center):
        """物の中心の高さで、付け根から見て手前side_pregraspの所から水平に差し込めるか確かめる。"""
        z = max(center[2], self.side_min_z)
        end = np.array([center[0], center[1], z])
        grasp = self.solve_side(end)
        if grasp is None:
            return None
        _, pitch, direction = grasp
        start = end - self.side_pregrasp * direction
        above = self.solve_side(start, [pitch], direction=direction)
        lifted = self.solve_side(end + [0.0, 0.0, self.side_lift], [pitch], direction=direction)
        if above is None or lifted is None:
            return None
        return {'side': True, 'z': z, 'pitch': pitch, 'direction': direction, 'start': start, 'end': end,
                'above': above[0]}

    def solve_side(self, position, pitches=None, previous=None, direction=None):
        seeds = [previous] if previous else []
        seeds.append({n: self.joints.get(n, 0.0) for n in self.arm_joints})
        n = len(self.arm_joints)
        seeds += [dict(zip(self.arm_joints, self.side_seeds[i:i + n])) for i in range(0, len(self.side_seeds), n)]
        return self.side_ik.solve(position, self.side_pitches if pitches is None else pitches, seeds, direction)

    def move_side_linear(self, start, end, pitch, direction):
        start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        steps = max(2, int(np.ceil(np.linalg.norm(end - start) / 0.01)) + 1)
        qs, previous = [], None
        for t in np.linspace(0.0, 1.0, steps):
            found = self.solve_side(start + t * (end - start), [pitch], previous, direction)
            if found is None:
                raise ArmFailed('planning', f'no side IK at {np.round(start + t * (end - start), 3)}')
            previous = found[0]
            qs.append(previous)
        self.follow(qs, np.linalg.norm(end - start) / (steps - 1))

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

    def plan_vertical(self, line, z, roll=None):
        """lineの真上pregrasp_heightから高さzまで真下向きに上下できるか確かめ、上の姿勢を返す。"""
        if line[0] < self.min_reach_x:
            return None
        roll = self.grasp_roll if roll is None else roll
        above_z = z + self.pregrasp_height
        down, above, lifted = (self.solve(line, h, roll=roll) for h in (z, above_z, z + self.lift_height))
        if down is None or above is None or lifted is None:
            return None
        return {'line': line, 'z': z, 'above_z': above_z, 'above': above, 'roll': roll}

    def solve(self, xy, z, previous=None, roll=None):
        seeds = [previous] if previous else []
        seeds.append({n: self.joints.get(n, 0.0) for n in self.arm_joints})
        n = len(self.arm_joints)
        seeds += [dict(zip(self.arm_joints, self.ik_seeds[i:i + n])) for i in range(0, len(self.ik_seeds), n)]
        return self.ik.solve([xy[0], xy[1], z], self.grasp_roll if roll is None else roll, seeds)

    def move_to(self, joints):
        code = move_joints(self.move_client, self.planning_group, self.arm_joints, joints, self.velocity_scaling)
        if code != MoveItErrorCodes.SUCCESS:
            raise ArmFailed('execution' if code in EXECUTION_ERRORS else 'planning',
                            f'move_group failed (error code {code})')

    def move_linear(self, xy, z_from, z_to, roll=None):
        # MoveItの直線補間はIKを使い、5関節のこのアームでは解けないため、ここで解いた点をつなぐ。
        steps = max(2, int(np.ceil(abs(z_to - z_from) / 0.01)) + 1)
        qs, previous = [], None
        for z in np.linspace(z_from, z_to, steps):
            q = self.solve(xy, z, previous, roll)
            if q is None:
                raise ArmFailed('planning', f'no IK at z={z:.3f}')
            previous = q
            qs.append(q)
        self.follow(qs, abs(z_to - z_from) / (steps - 1))

    def follow(self, qs, step_length):
        """解いた点の並びを、点の間隔step_length[m]をlinear_speedで進む速さでつなぎ、コントローラへ直接送る。"""
        dt = step_length / self.linear_speed
        points = []
        for i, q in enumerate(qs):
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
