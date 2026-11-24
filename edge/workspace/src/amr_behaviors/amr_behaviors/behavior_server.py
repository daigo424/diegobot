import copy
import math
import threading
import time

import numpy as np
import rclpy
from amr_interfaces.action import ApproachObject, SearchObject
from amr_interfaces.srv import FindObjects, ResolveObject
from amr_interfaces.msg import DetectedObject
from amr_manipulation.ros_util import call, find_objects, move_joints, resolve_object
from amr_manipulation.top_down_ik import Gaze, ViewPose
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetStateValidity
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
import tf2_geometry_msgs  # noqa: F401  PointStamped・PoseStampedをBuffer.transformで変換できるよう登録する


class Canceled(Exception):
    pass


class Failed(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BehaviorServer(Node):
    """車体の動き(その場の回転・前進)とアーム・カメラを組み合わせた、探す・近づく動作。"""

    def __init__(self):
        super().__init__('behavior_server')
        p = self.declare_parameter
        self.arm_joints = p('arm_joints', ['']).value
        self.search_poses = p('search_poses', [0.0]).value
        self.gaze_pose = p('gaze_pose', [0.0]).value
        self.gaze_joints = p('gaze_joints', ['']).value
        self.gaze_limits = {'wrist_flex': tuple(p('gaze_wrist_limits', [0.3, 1.45]).value),
                            'shoulder_pan': tuple(p('gaze_pan_limits', [-3.15, 0.5]).value)}
        self.gaze_period = p('gaze_period', 0.2).value
        self.camera_link = p('camera_link', 'camera_camera_link_0').value
        self.base_frame = p('base_frame', 'base_link').value
        self.planning_group = p('planning_group', 'arm_group').value
        self.velocity_scaling = p('velocity_scaling', 0.5).value
        self.settle_time = p('settle_time', 1.5).value
        self.find_attempts = p('find_attempts', 4).value
        self.search_min_score = p('search_min_score', 0.15).value
        self.odom_frame = p('odom_frame', 'odom').value
        self.standoff = p('standoff', 0.18).value
        self.side_ratio = p('side_ratio', 1.2).value
        self.side_target = p('side_target', [0.29, 0.069]).value
        # 名前で指定された物は、見分けるのに向いた距離で止まり、手首カメラをview_poseの姿勢にしてVLMに見せてから近づく。
        self.view_standoff = p('view_standoff', 0.45).value
        self.view_seeds = p('view_seeds', [0.0]).value
        self.view_angles = np.radians(p('view_angles', [10.0]).value)
        self.view_roll = p('view_roll', -1.5708).value
        self.view_max_shoulder = p('view_max_shoulder', 1.575).value
        self.view_min_elbow = p('view_min_elbow', -0.005).value
        self.max_candidates = p('max_candidates', 4).value
        self.exclude_radius = p('exclude_radius', 0.08).value
        self.floor_z = p('floor_z', -0.00945).value
        self.resolve_timeout = p('resolve_timeout', 90.0).value
        self.rotation_center_x = p('rotation_center_x', 0.035).value
        self.track_gate = p('track_gate', 0.25).value
        self.track_max_step = p('track_max_step', 0.08).value
        self.track_min_gate = p('track_min_gate', 0.04).value
        self.view_sag = p('view_sag', 0.1).value
        self.lost_timeout = p('lost_timeout', 3.0).value
        self.approach_timeout = p('approach_timeout', 90.0).value
        self.turn_speed = p('turn_speed', 1.0).value
        self.min_turn_speed = p('min_turn_speed', 0.7).value
        self.turn_in_place_angle = p('turn_in_place_angle', 0.2).value
        self.yaw_tolerance = p('yaw_tolerance', 0.05).value
        self.drive_speed = p('drive_speed', 0.16).value
        self.min_drive_speed = p('min_drive_speed', 0.11).value
        self.steer_gain = p('steer_gain', 1.5).value
        self.distance_tolerance = p('distance_tolerance', 0.015).value
        self.front_x = p('front_x', 0.079).value
        self.half_width = p('half_width', 0.17).value
        self.stop_distance = p('stop_distance', 0.12).value
        self.max_drive = p('max_drive', 1.5).value
        self.sighting_max_age = p('sighting_max_age', 600.0).value
        self.sighting_max_offset = p('sighting_max_offset', 0.2).value
        # 探す・物を見続けるカメラ。headなら頭部のパン・チルトで見回し、アームはhead_arm_poseでよけておく。
        self.view = p('view_camera', 'head').value
        self.head_joints = p('head_joints', ['head_pan', 'head_tilt']).value
        self.head_camera_link = p('head_camera_link', 'head_camera_link').value
        self.head_search_poses = p('head_search_poses', [0.0, -0.45]).value
        self.head_arm_pose = p('head_arm_pose', [-0.6, -1.65, 1.4, 0.8519, -1.5708]).value
        self.head_speed = p('head_speed', 1.0).value
        find_service = {'head': p('head_find_service', '/head_object_finder/find').value,
                        'wrist': p('wrist_find_service', '/object_finder/find').value}[self.view]

        group = ReentrantCallbackGroup()
        self.odom = None
        self.scan = None
        self.gaze = None
        self.joints = {}
        self.busy = False
        self.sighting = None
        self.rejected = []
        self.lock = threading.Lock()
        self.tf_buffer = Buffer()
        # 検出を待つ間もTFを受け取り続けるため、専用ノード(node=None)を別スレッドで回す。
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10, callback_group=group)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.on_description, latched, callback_group=group)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10, callback_group=group)
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_group_controller/follow_joint_trajectory',
                                       callback_group=group)
        self.head_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory',
                                        callback_group=group)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data, callback_group=group)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.find_client = self.create_client(FindObjects, find_service, callback_group=group)
        self.move_client = ActionClient(self, MoveGroup, '/move_action', callback_group=group)
        self.resolve_client = self.create_client(ResolveObject, p('resolve_service', '/object_resolver/resolve').value,
                                                 callback_group=group)
        self.validity_client = self.create_client(GetStateValidity, '/check_state_validity', callback_group=group)
        self.wrist_info = None
        self.view_pose = None
        self.create_subscription(CameraInfo, p('wrist_camera_info', '/camera/camera/color/camera_info').value,
                                 self.on_wrist_info, 1, callback_group=group)
        cancel = lambda _: CancelResponse.ACCEPT  # noqa: E731
        ActionServer(self, SearchObject, '~/search', self.execute_search, goal_callback=self.on_goal,
                     cancel_callback=cancel, callback_group=group)
        ActionServer(self, ApproachObject, '~/approach', self.execute_approach, goal_callback=self.on_goal,
                     cancel_callback=cancel, callback_group=group)

    def on_odom(self, msg):
        self.odom = msg

    def on_scan(self, msg):
        self.scan = msg

    def on_wrist_info(self, msg):
        self.wrist_info = msg

    def on_description(self, msg):
        self.view_pose = ViewPose(msg.data, self.base_frame, self.camera_link, 'gripper_frame_link',
                                  {'wrist_roll': self.view_roll})
        if self.view == 'head':
            self.gaze = Gaze(msg.data, self.base_frame, self.head_camera_link, self.head_joints, optical=False)
        else:
            self.gaze = Gaze(msg.data, self.base_frame, self.camera_link, self.gaze_joints)

    def on_joint_states(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def on_goal(self, _):
        # 車体とアームは1つなので、実行中の依頼があれば新しい依頼は断る。
        with self.lock:
            if self.busy:
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    # --- 探す ---

    def execute_search(self, goal_handle):
        req, result = goal_handle.request, SearchObject.Result()
        try:
            obj, message = self.search(goal_handle, req.queries, req.color,
                                       lambda stage, heading: SearchObject.Feedback(stage=stage, heading=heading))
            result.message = message
            if obj is None:
                result.error_code = SearchObject.Result.NOT_FOUND
                goal_handle.abort()
            else:
                result.error_code, result.object = SearchObject.Result.SUCCESS, obj
                goal_handle.succeed()
        except Canceled as e:
            result.error_code, result.message = SearchObject.Result.CANCELED, str(e)
            goal_handle.canceled()
        except Failed as e:
            result.error_code, result.message = SearchObject.Result.FAILED, str(e)
            goal_handle.abort()
        finally:
            self.stop()
            self.busy = False
        return result

    def find_candidate(self, goal_handle, queries, color, exclude, fb):
        """前に見つけた場所か見回して、除外の位置から離れた物を探し、(odom上の位置, 説明)を返す。"""
        obj, message = self.look_at_sighting(queries, color, exclude), ''
        if obj is None:
            obj, message = self.search(goal_handle, queries, color, lambda s, _: fb('looking'), exclude)
        else:
            self.get_logger().info('found the object again at the last sighting')
        return (self.to_odom(obj) if obj is not None else None), message

    def identify(self, name, queries, target, exclude):
        """手首カメラを物を見分けやすい姿勢にし、VLM(object_resolver)にnameの物かを確かめさせる。
        対象ならそのodom上の位置、違えばNoneを返す。見分けられなければFailed。"""
        if self.view_pose is None or self.wrist_info is None:
            raise Failed(ApproachObject.Result.AMBIGUOUS, 'robot_description or the wrist camera info is not received')
        lx, ly, _ = self.to_base(target, self.current_pose())
        bottom, height, radius = (lx, ly, self.floor_z), 0.09, 0.035
        # 隣の物(違うと分かった物を含む)の形を使うと、そちらへカメラを向けてしまうため、追っている位置のすぐ近くだけ見る。
        shape = self.measure_shape(queries, '', target, exclude, self.exclude_radius)
        self.get_logger().info(f'identifying the candidate at ({target[0]:.3f}, {target[1]:.3f}) in odom '
                               f'({"with" if shape is not None else "without"} its shape), excluding '
                               f'{[(round(x, 3), round(y, 3)) for x, y in exclude]}')
        if shape is not None:
            c, e = shape.obb_pose.pose.position, shape.obb_extents
            bottom, height, radius = (c.x, c.y, self.floor_z), max(e.z, 0.05), max(e.x, e.y, 0.04) / 2 + 0.01
        n = 4
        seeds = [dict(zip(self.arm_joints[:n], self.view_seeds[i:i + n])) for i in range(0, len(self.view_seeds), n)]
        k = self.wrist_info.k
        q, angle = self.view_pose.solve(bottom, height, radius, seeds, (self.wrist_info.width, self.wrist_info.height),
                                        (k[0], k[4]), self.view_angles, floor_z=self.floor_z,
                                        limits={'shoulder_lift': (-np.inf, self.view_max_shoulder),
                                                'elbow_flex': (self.view_min_elbow, np.inf)},
                                        accept=self.state_valid)
        if q is None:
            raise Failed(ApproachObject.Result.AMBIGUOUS, 'no arm pose to look at the candidate')
        self.get_logger().info(f'looking at the candidate from {np.degrees(angle):.0f} deg above')
        self.arm_to([q[j] for j in self.arm_joints])
        time.sleep(self.settle_time)
        self.rejected = []
        response = resolve_object(self.resolve_client, name, queries,
                                  [Point(x=float(x), y=float(y)) for x, y in exclude], self.resolve_timeout)
        # 頭部カメラで見続けられるよう、アームをよける。
        self.prepare_view()
        if response is None:
            raise Failed(ApproachObject.Result.AMBIGUOUS, 'object_resolver did not answer')
        self.get_logger().info(f'identification: {response.message}')
        codes = ResolveObject.Response
        if response.error_code == codes.SUCCESS:
            return self.to_odom(response.object) or target
        if response.error_code in (codes.NO_MATCH, codes.NOT_FOUND, codes.TOO_FAR):
            # 一緒に写っていてVLMが違うと答えた物も、次の候補にしない。
            self.rejected = [p[:2] for p in (self.to_odom(o) for o in response.checked) if p is not None]
            self.get_logger().info(f'rejected {[(round(x, 3), round(y, 3)) for x, y in self.rejected]} in odom')
            return None
        raise Failed(ApproachObject.Result.AMBIGUOUS, response.message)

    def state_valid(self, q):
        """MoveItの判定で、関節角度qのアームが車体や自分に当たらないか。腕を前へ伸ばすと重さで肩が前へ下がるため、
        肩をview_sagだけ下げた姿勢でも当たらないことを確かめる。"""
        names = list(self.arm_joints)
        for sag in (0.0, self.view_sag):
            moved = dict(q, shoulder_lift=q['shoulder_lift'] + sag)
            js = JointState(name=names, position=[float(moved[n]) for n in names])
            response = call(self.validity_client, GetStateValidity.Request(
                robot_state=RobotState(joint_state=js, is_diff=True), group_name=self.planning_group), 5.0)
            if response is None or not response.valid:
                return False
        return True

    def excluded(self, obj, exclude):
        p = self.to_odom(obj)
        return p is not None and any(math.dist(p[:2], e) <= self.exclude_radius for e in exclude)

    def search(self, goal_handle, queries, color, feedback, exclude=()):
        """見つかれば(物体, 説明)、見つからなければ(None, 説明)を返す。"""
        self.prepare_view()
        if self.view == 'head':
            values, joints = self.head_search_poses, self.head_joints
        else:
            values, joints = self.search_poses, self.arm_joints
        n = len(joints)
        poses = [dict(zip(joints, values[i:i + n])) for i in range(0, len(values), n)]
        return self.look_through(goal_handle, poses, queries, color, feedback, exclude)

    def look_through(self, goal_handle, poses, queries, color, feedback, exclude=()):
        message = ''
        for pose in poses:
            self.check_cancel(goal_handle)
            # 付け根・頭のパン(先頭の関節)は正の向きでカメラが右(base_linkの−y)を向く。
            goal_handle.publish_feedback(feedback('looking', -next(iter(pose.values()))))
            try:
                self.view_to(pose)
            except Failed as e:
                self.get_logger().warn(f'skip search pose {pose}: {e}')
                continue
            time.sleep(self.settle_time)
            objects, message = find_objects(self.find_client, queries, color, self.find_attempts,
                                            self.search_min_score)
            objects = [o for o in objects if not self.excluded(o, exclude)]
            if objects:
                self.remember(objects[0])
                p = objects[0].position.point
                return objects[0], f'found {objects[0].label} at ({p.x:.2f}, {p.y:.2f})'
        return None, message or f'no {color} {"/".join(queries)}'.strip()

    def remember(self, obj):
        p = self.to_odom(obj)
        if p is not None:
            self.sighting = (p, time.time())

    def look_at_sighting(self, queries, color, exclude=()):
        """前に見つけた場所へカメラを向け、その近くに同じ条件の物があれば返す。無ければNone。"""
        if self.sighting is None or self.gaze is None or time.time() - self.sighting[1] > self.sighting_max_age:
            return None
        target = self.sighting[0]
        self.prepare_view()
        q, offset = self.aim_with_offset(target, self.current_pose(), self.view_seed())
        # 動かせる関節だけでは写せない向き(車体の後ろ等)なら見回しに任せる。
        if np.max(np.abs(offset)) > self.sighting_max_offset:
            return None
        try:
            self.view_to(q)
        except Failed as e:
            self.get_logger().warn(f'cannot look at the last sighting: {e}')
            return None
        time.sleep(self.settle_time)
        objects, _ = find_objects(self.find_client, queries, color, self.find_attempts, self.search_min_score)
        obj = self.nearest([o for o in objects if not self.excluded(o, exclude)], target)
        if obj is None:
            self.get_logger().info(f'not at the last sighting ({len(objects)} matching objects elsewhere)')
        return obj

    def nearest(self, objects, target, gate=None):
        """odom上のtargetからgate(既定はtrack_gate)以内で一番近い検出。無ければNone。"""
        gate = self.track_gate if gate is None else gate
        near = [(math.dist(p[:2], target[:2]), o) for o in objects if (p := self.to_odom(o)) is not None]
        near = [(d, o) for d, o in near if d <= gate]
        return min(near, key=lambda x: x[0])[1] if near else None

    # --- 近づく ---

    def execute_approach(self, goal_handle):
        req, result = goal_handle.request, ApproachObject.Result()
        standoff = req.standoff if req.standoff > 0.0 else self.standoff
        tracker = None
        try:
            fb = lambda stage, remaining=0.0: ApproachObject.Feedback(stage=stage, remaining=remaining)  # noqa: E731
            name = req.target_name.strip()
            # 名前で指定されたときは色で絞らない(見分けはVLMに任せ、色の判定で候補を落とさない)。
            color = '' if name else req.color
            exclude = [(e.x, e.y) for e in req.exclude]
            unseen = []
            for attempt in range(self.max_candidates if name else 1):
                goal_handle.publish_feedback(fb('looking'))
                target, message = self.find_candidate(goal_handle, req.queries, color, exclude, fb)
                if target is None:
                    code = ApproachObject.Result.NO_MATCH if name and attempt else ApproachObject.Result.NOT_FOUND
                    raise Failed(code, message or 'cannot locate the object in odom')
                if not name:
                    break
                tracker = Tracker(self, req.queries, color, target, exclude)
                self.drive_to(goal_handle, tracker, self.view_standoff, fb)
                tracker.stop()
                goal_handle.publish_feedback(fb('identifying'))
                found = self.identify(name, req.queries, tracker.target, exclude)
                if found is not None:
                    target = found
                    break
                # VLMが実際に確かめた物だけを除く。追っていた候補が手首カメラに写っていなかったときは、もう一度見に行き、
                # 2回続けて写らなければ除く(見ていない物を違うとして除くと、それが対象だったときに見つからない)。
                exclude += self.rejected
                seen = any(math.dist(tracker.target[:2], r) <= self.exclude_radius for r in self.rejected)
                if not seen and any(math.dist(tracker.target[:2], u) <= self.exclude_radius for u in unseen):
                    exclude.append(tracker.target[:2])
                elif not seen:
                    unseen.append(tracker.target[:2])
                    self.get_logger().info('the candidate was not in the wrist camera view; will look at it again')
                self.sighting = None
            else:
                raise Failed(ApproachObject.Result.NO_MATCH,
                             f'none of {self.max_candidates} candidate(s) is "{name}"')
            # 縦に長い物はmanipulatorが横からつかむ。アームの正面の決まった距離にしか届かないため、そこへ止まる。
            lateral = 0.0
            shape = self.measure_shape(req.queries, color, target)
            if req.standoff <= 0.0 and shape is not None and self.is_tall(shape):
                standoff, lateral = self.side_target
                self.get_logger().info(f'tall object; stopping with it at ({standoff:.3f}, {lateral:.3f}) from base_link')
            tracker = Tracker(self, req.queries, color, target, exclude)
            self.drive_to(goal_handle, tracker, standoff, fb, lateral)
            tracker.stop()
            lx, ly, _ = self.to_base(tracker.target, self.current_pose())
            self.get_logger().info(f'stopped with the object estimated at ({lx:.3f}, {ly:.3f}) from base_link')
            # 止まった位置で測り直し、呼び出し側(つかむ等)が使える位置を返す。カメラは物体へ向いたままにする
            # (姿勢を変えると写り方と明るさが変わり、見えていた物を見落とすことがある)。
            time.sleep(self.settle_time)
            objects, message = find_objects(self.find_client, req.queries, color, self.find_attempts,
                                            self.search_min_score)
            # 確信度の高い順では、近くの別の物を返すことがあるため、追ってきた物に一番近いものを返す。
            obj = self.nearest(objects, tracker.target)
            if obj is None:
                raise Failed(ApproachObject.Result.NOT_FOUND,
                             f'lost after approaching: {message or "only other objects are in view"}')
            self.remember(obj)
            measured = self.measure_shape(req.queries, color, tracker.target) or obj
            result.object_odom = self.object_in_odom(measured)
            result.error_code, result.object = ApproachObject.Result.SUCCESS, obj
            p = obj.position.point
            result.message = f'{obj.label} is at ({p.x:.2f}, {p.y:.2f})'
            goal_handle.succeed()
        except Failed as e:
            result.error_code, result.message = e.code, str(e)
            self.get_logger().warn(f'approach failed: {e}')
            goal_handle.abort()
        except Canceled as e:
            result.error_code, result.message = ApproachObject.Result.CANCELED, str(e)
            goal_handle.canceled()
        finally:
            if tracker is not None:
                tracker.stop()
            self.stop()
            self.busy = False
        return result

    def measure_shape(self, queries, color, target, exclude=(), gate=None):
        """odom上のtargetの近く(gate以内、既定はtrack_gate)にある物を、形(OBB)付きで測り直す。測れなければNone。"""
        for _ in range(self.find_attempts):
            response = call(self.find_client, FindObjects.Request(
                queries=[f'{color} {q}'.strip() for q in queries], min_score=float(self.search_min_score),
                want_obb=True), 15.0)
            if response is not None and response.success:
                obj = self.nearest([o for o in response.objects if o.position_valid and o.obb_valid
                                    and not self.excluded(o, exclude)], target, gate)
                if obj is not None:
                    return obj
            time.sleep(0.5)
        return None

    def is_tall(self, obj):
        e = obj.obb_extents
        return obj.obb_valid and e.z > self.side_ratio * e.x

    def object_in_odom(self, obj):
        """位置とOBBをodomへ直した複製。つかむまでの間に車体が動いても使えるよう、呼び出し側へはこちらを渡す。"""
        out = copy.deepcopy(obj)
        try:
            out.position = self.tf_buffer.transform(obj.position, self.odom_frame, timeout=Duration(seconds=0.3))
            if obj.obb_valid:
                out.obb_pose = self.tf_buffer.transform(obj.obb_pose, self.odom_frame, timeout=Duration(seconds=0.3))
        except TransformException as e:
            self.get_logger().warn(f'cannot transform the object to {self.odom_frame}: {e}')
            return DetectedObject()
        return out

    def drive_to(self, goal_handle, tracker, standoff, fb, lateral=0.0):
        """カメラを物体へ向け続けながら、車軸の中点から見た物体がstandoffの位置・正面からlateralだけ左に来るまで動く。"""
        if self.gaze is None:
            raise Failed(ApproachObject.Result.DRIVE_FAILED, 'robot_description not received yet')
        # 探す姿勢からカメラを物体へ向ける姿勢へは、アームなら車体やLiDARに当たらないようMoveItで移る。以降は少しずつ直接送る。
        gaze = self.aim(tracker.target, self.current_pose(), self.view_seed())
        self.view_to(gaze)
        tracker.forget_until_now()
        start, traveled, deadline = self.current_pose(), 0.0, time.time() + self.approach_timeout
        previous, next_gaze, phase = start, 0.0, None
        while True:
            self.check_cancel(goal_handle)
            if time.time() > deadline:
                raise Failed(ApproachObject.Result.DRIVE_FAILED, 'approach timed out')
            x, y, yaw = self.current_pose()
            traveled += math.hypot(x - previous[0], y - previous[1])
            previous = (x, y, yaw)
            if traveled > self.max_drive:
                raise Failed(ApproachObject.Result.DRIVE_FAILED, f'drove more than {self.max_drive} m')
            lx, ly, _ = self.to_base(tracker.target, (x, y, yaw))
            # 差動二輪は車軸の中点の周りに回るため、向きと距離はそこから測る。
            dx = lx - self.rotation_center_x
            # 物体までの距離と向きを、止まりたい位置での値に合わせる(その場で回ると向きだけが、進むと主に距離が変わる)。
            heading = math.atan2(ly, dx) - math.atan2(lateral, standoff - self.rotation_center_x)
            remaining = math.hypot(dx, ly) - math.hypot(standoff - self.rotation_center_x, lateral)
            goal_handle.publish_feedback(fb('driving', remaining))
            if time.time() >= next_gaze:
                gaze = self.aim(tracker.target, (x, y, yaw), gaze)
                self.stream_view(gaze)
                next_gaze = time.time() + self.gaze_period
            if time.time() - tracker.last_seen > self.lost_timeout:
                raise Failed(ApproachObject.Result.NOT_FOUND, 'lost the object while approaching')

            arrived = abs(remaining) <= self.distance_tolerance and abs(heading) <= self.yaw_tolerance
            turning = abs(heading) > self.turn_in_place_angle or abs(remaining) <= self.distance_tolerance
            now = 'arrived' if arrived else 'turning' if turning else 'driving'
            if now != phase:
                # その場の回転と前進が切り替わると車体が止まって見えるため、切り替わりの理由が分かるよう記録する。
                self.get_logger().info(
                    f'{now}: object at ({lx:.3f}, {ly:.3f}) from base_link, remaining {remaining:.3f} m, '
                    f'heading {math.degrees(heading):.1f} deg, traveled {traveled:.2f} m, '
                    f'last seen {time.time() - tracker.last_seen:.1f} s ago')
                phase = now
            if arrived:
                break
            if turning:
                # 遅すぎる指令ではモーターが回らないため、最低速度を守る(値の根拠は設定ファイル)。
                speed = min(self.turn_speed, max(self.min_turn_speed, 1.5 * abs(heading)))
                self.publish(0.0, math.copysign(speed, heading))
            else:
                if remaining > 0 and self.obstacle_ahead():
                    raise Failed(ApproachObject.Result.DRIVE_FAILED, 'obstacle ahead')
                speed = min(self.drive_speed, max(self.min_drive_speed, 1.5 * abs(remaining)))
                self.publish(math.copysign(speed, remaining), self.steer_gain * heading)
            time.sleep(0.05)
        self.stop()

    def to_base(self, target, pose):
        x, y, yaw = pose
        tx, ty, tz = target
        return (math.cos(yaw) * (tx - x) + math.sin(yaw) * (ty - y),
                -math.sin(yaw) * (tx - x) + math.cos(yaw) * (ty - y), tz)

    def aim(self, target, pose, seed):
        return self.aim_with_offset(target, pose, seed)[0]

    def aim_with_offset(self, target, pose, seed):
        """odom上の物体へカメラの光軸を向ける関節角度と、残る画像上のずれ。
        アームなら付け根と手首だけ、頭部ならパンとチルトを動かす。"""
        if self.view == 'head':
            return self.gaze.solve(self.to_base(target, pose), {}, seed)
        fixed = dict(zip(self.arm_joints, self.gaze_pose))
        return self.gaze.solve(self.to_base(target, pose), fixed, seed, limits=self.gaze_limits)

    def view_seed(self):
        if self.view == 'head':
            return {n: self.joints.get(n, 0.0) for n in self.head_joints}
        return dict(zip(self.arm_joints, self.gaze_pose))

    def prepare_view(self):
        """頭部カメラで見るときは、アームが写り込んで深度が抜けないよう横へよけておく。"""
        if self.view != 'head':
            return
        current = [self.joints.get(n) for n in self.arm_joints]
        if None not in current and max(abs(a - b) for a, b in zip(current, self.head_arm_pose)) < 0.05:
            return
        self.arm_to(self.head_arm_pose)

    def view_to(self, joints):
        if self.view == 'head':
            self.head_to(joints)
        else:
            self.arm_to([joints[n] for n in self.arm_joints])

    def stream_view(self, joints):
        # 前の目標は新しい目標で置き換わる(JTCが受け付けた時点で前の軌道を捨てる)ため、結果は待たない。
        names = self.head_joints if self.view == 'head' else self.arm_joints
        client = self.head_client if self.view == 'head' else self.arm_client
        client.send_goal_async(self.trajectory(names, joints, 2.0 * self.gaze_period))

    def trajectory(self, names, joints, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        sec = int(seconds)
        goal.trajectory.points = [JointTrajectoryPoint(positions=[float(joints[n]) for n in names],
                                                       time_from_start=DurationMsg(sec=sec, nanosec=int((seconds - sec) * 1e9)))]
        return goal

    def head_to(self, joints):
        """頭部を指定の角度へ動かし、着くまで待つ。頭部はアームに当たらない範囲しか動かないため、MoveItを通さない。"""
        if not self.head_client.wait_for_server(timeout_sec=5.0):
            raise Failed(SearchObject.Result.FAILED, 'head controller is not available')
        distance = max((abs(joints[n] - self.joints.get(n, 0.0)) for n in self.head_joints), default=0.0)
        seconds = max(0.5, distance / self.head_speed)
        done = threading.Event()
        state = {}

        def on_result(future):
            state['result'] = future.result()
            done.set()

        def on_goal(future):
            handle = future.result()
            if not handle.accepted:
                done.set()
                return
            handle.get_result_async().add_done_callback(on_result)
        self.head_client.send_goal_async(self.trajectory(self.head_joints, joints, seconds)).add_done_callback(on_goal)
        if not done.wait(seconds + 5.0) or 'result' not in state or state['result'].result.error_code != 0:
            raise Failed(SearchObject.Result.FAILED, f'head move to {joints} failed')

    def to_odom(self, obj):
        """物体の位置を、撮った時刻の車体の位置でodomへ直す(画像の遅れの間に車体が動いてもずれない)。"""
        try:
            p = self.tf_buffer.transform(obj.position, self.odom_frame, timeout=Duration(seconds=0.3))
        except TransformException as e:
            self.get_logger().warn(f'cannot transform to {self.odom_frame}: {e}')
            return None
        return p.point.x, p.point.y, p.point.z

    # --- 車体とアームの低レベルの動き ---

    def arm_to(self, joints):
        code = move_joints(self.move_client, self.planning_group, self.arm_joints,
                           dict(zip(self.arm_joints, joints)), self.velocity_scaling)
        if code != MoveItErrorCodes.SUCCESS:
            raise Failed(SearchObject.Result.FAILED, f'arm move failed (MoveIt error code {code})')

    def obstacle_ahead(self):
        # LiDARの高さ(天板の上)より低い物は見えないので、壁や家具など背の高い物だけ止まれる。
        scan = self.scan
        if scan is None or (self.get_clock().now() - Time.from_msg(scan.header.stamp)).nanoseconds > 1e9:
            return True
        r = np.asarray(scan.ranges)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > scan.range_min)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        return bool(np.any((x > self.front_x) & (x < self.front_x + self.stop_distance) & (np.abs(y) < self.half_width)))

    def current_pose(self):
        odom = self.odom
        if odom is None or (self.get_clock().now() - Time.from_msg(odom.header.stamp)).nanoseconds > 1e9:
            raise Failed(ApproachObject.Result.DRIVE_FAILED, 'no recent /odometry/filtered')
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        return p.x, p.y, yaw_of(q)

    def publish(self, linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)

    def check_cancel(self, goal_handle):
        if goal_handle.is_cancel_requested:
            self.stop()
            raise Canceled('canceled')


class Tracker:
    """別スレッドでobject_finderを呼び続け、追っている物体のodom上の位置を更新する。"""

    def __init__(self, node, queries, color, target, exclude=()):
        self.node, self.queries, self.color = node, queries, color
        self.target = target
        self.exclude = list(exclude)
        self.pending = None
        self.last_seen = time.time()
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def forget_until_now(self):
        # アームを動かした直後は、見失った扱いにしない。
        self.last_seen = time.time()

    def run(self):
        while self.running:
            objects, message = find_objects(self.node.find_client, self.queries, self.color, 1,
                                            self.node.search_min_score)
            # 対象ではないと分かった物の近くの検出は使わない(近くに並べた物の間で乗り移らないため)。
            points = [p for p in (self.node.to_odom(o) for o in objects) if p is not None and not any(
                math.dist(p[:2], e) <= self.node.exclude_radius for e in self.exclude)]
            points.sort(key=lambda p: math.dist(p[:2], self.target[:2]))
            # 別の同じような物に乗り換えないよう、今の推定の近くにある物だけを使う。近くに別の物が写っているときは、
            # その物までの距離の半分までに狭める。
            gate = self.node.track_gate
            if len(points) > 1:
                gate = min(gate, max(self.node.track_min_gate, math.dist(points[1][:2], self.target[:2]) / 2.0))
            best = points[0] if points and math.dist(points[0][:2], self.target[:2]) <= gate else None
            if best is not None:
                step = math.dist(best[:2], self.target[:2])
                # 置かれた物は動かないため、大きく飛んだ検出は深度の誤り等とみなし、続けて同じ所に出たときだけ信じる
                # (1回の誤った検出で目標が奥へずれると、止まる手前で走り出して物を蹴る)。
                if step > self.node.track_max_step and (
                        self.pending is None or math.dist(best[:2], self.pending[:2]) > self.node.track_max_step):
                    self.node.get_logger().info(f'tracking: ignored a jump of {step:.3f} m in odom')
                    self.pending = best
                    best = None
            if best is not None:
                self.node.get_logger().info(f'tracking: moved {step:.3f} m in odom', throttle_duration_sec=1.0)
                self.target, self.pending = best, None
                self.last_seen = time.time()
            else:
                self.node.get_logger().info(
                    f'tracking: no update ({len(objects)} detections, {message or "all outside the gate"})',
                    throttle_duration_sec=1.0)
            time.sleep(0.1)

    def stop(self):
        self.running = False


def main():
    rclpy.init()
    node = BehaviorServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
