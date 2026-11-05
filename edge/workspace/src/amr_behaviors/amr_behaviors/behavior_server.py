import math
import threading
import time

import numpy as np
import rclpy
from amr_interfaces.action import ApproachObject, SearchObject
from amr_interfaces.srv import FindObjects
from amr_manipulation.ros_util import find_objects, move_joints
from amr_manipulation.top_down_ik import Gaze
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
import tf2_geometry_msgs  # noqa: F401  PointStampedをBuffer.transformで変換できるよう登録する


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
        self.gaze_wrist_limits = p('gaze_wrist_limits', [0.3, 1.45]).value
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
        self.rotation_center_x = p('rotation_center_x', 0.035).value
        self.track_gate = p('track_gate', 0.25).value
        self.track_max_step = p('track_max_step', 0.08).value
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

        group = ReentrantCallbackGroup()
        self.odom = None
        self.scan = None
        self.gaze = None
        self.joints = {}
        self.busy = False
        self.sighting = None
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
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data, callback_group=group)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.find_client = self.create_client(FindObjects, '/object_finder/find', callback_group=group)
        self.move_client = ActionClient(self, MoveGroup, '/move_action', callback_group=group)
        cancel = lambda _: CancelResponse.ACCEPT  # noqa: E731
        ActionServer(self, SearchObject, '~/search', self.execute_search, goal_callback=self.on_goal,
                     cancel_callback=cancel, callback_group=group)
        ActionServer(self, ApproachObject, '~/approach', self.execute_approach, goal_callback=self.on_goal,
                     cancel_callback=cancel, callback_group=group)

    def on_odom(self, msg):
        self.odom = msg

    def on_scan(self, msg):
        self.scan = msg

    def on_description(self, msg):
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

    def search(self, goal_handle, queries, color, feedback):
        """見つかれば(物体, 説明)、見つからなければ(None, 説明)を返す。"""
        n = len(self.arm_joints)
        poses = [self.search_poses[i:i + n] for i in range(0, len(self.search_poses), n)]
        return self.look_through(goal_handle, poses, queries, color, feedback)

    def look_through(self, goal_handle, poses, queries, color, feedback):
        message = ''
        for pose in poses:
            self.check_cancel(goal_handle)
            # 付け根(先頭の関節)は正の向きでカメラが右(base_linkの−y)を向く。
            goal_handle.publish_feedback(feedback('looking', -pose[0]))
            try:
                self.arm_to(pose)
            except Failed as e:
                self.get_logger().warn(f'skip search pose {pose}: {e}')
                continue
            time.sleep(self.settle_time)
            objects, message = find_objects(self.find_client, queries, color, self.find_attempts,
                                            self.search_min_score)
            if objects:
                self.remember(objects[0])
                p = objects[0].position.point
                return objects[0], f'found {objects[0].label} at ({p.x:.2f}, {p.y:.2f})'
        return None, message or f'no {color} {"/".join(queries)}'.strip()

    def remember(self, obj):
        p = self.to_odom(obj)
        if p is not None:
            self.sighting = (p, time.time())

    def look_at_sighting(self, queries, color):
        """前に見つけた場所へカメラを向け、その近くに同じ条件の物があれば返す。無ければNone。"""
        if self.sighting is None or self.gaze is None or time.time() - self.sighting[1] > self.sighting_max_age:
            return None
        target = self.sighting[0]
        pose = dict(zip(self.arm_joints, self.gaze_pose))
        q, offset = self.gaze.solve(self.to_base(target, self.current_pose()), pose, pose,
                                    limits={'wrist_flex': tuple(self.gaze_wrist_limits)})
        # 付け根と手首だけでは写せない向き(車体の後ろ等)なら見回しに任せる。
        if np.max(np.abs(offset)) > self.sighting_max_offset:
            return None
        try:
            self.arm_to([q[n] for n in self.arm_joints])
        except Failed as e:
            self.get_logger().warn(f'cannot look at the last sighting: {e}')
            return None
        time.sleep(self.settle_time)
        objects, _ = find_objects(self.find_client, queries, color, self.find_attempts, self.search_min_score)
        obj = self.nearest(objects, target)
        if obj is None:
            self.get_logger().info(f'not at the last sighting ({len(objects)} matching objects elsewhere)')
        return obj

    def nearest(self, objects, target):
        """odom上のtargetからtrack_gate以内で一番近い検出。無ければNone。"""
        near = [(math.dist(p[:2], target[:2]), o) for o in objects if (p := self.to_odom(o)) is not None]
        near = [(d, o) for d, o in near if d <= self.track_gate]
        return min(near, key=lambda x: x[0])[1] if near else None

    # --- 近づく ---

    def execute_approach(self, goal_handle):
        req, result = goal_handle.request, ApproachObject.Result()
        standoff = req.standoff if req.standoff > 0.0 else self.standoff
        tracker = None
        try:
            fb = lambda stage, remaining=0.0: ApproachObject.Feedback(stage=stage, remaining=remaining)  # noqa: E731
            goal_handle.publish_feedback(fb('looking'))
            obj, message = self.look_at_sighting(req.queries, req.color), ''
            if obj is None:
                obj, message = self.search(goal_handle, req.queries, req.color, lambda s, _: fb('looking'))
            else:
                self.get_logger().info('found the object again at the last sighting')
            target = self.to_odom(obj) if obj is not None else None
            if target is None:
                raise Failed(ApproachObject.Result.NOT_FOUND, message or 'cannot locate the object in odom')
            tracker = Tracker(self, req.queries, req.color, target)
            self.drive_to(goal_handle, tracker, standoff, fb)
            tracker.stop()
            lx, ly, _ = self.to_base(tracker.target, self.current_pose())
            self.get_logger().info(f'stopped with the object estimated at ({lx:.3f}, {ly:.3f}) from base_link')
            # 止まった位置で測り直し、呼び出し側(つかむ等)が使える位置を返す。カメラは物体へ向いたままにする
            # (姿勢を変えると写り方と明るさが変わり、見えていた物を見落とすことがある)。
            time.sleep(self.settle_time)
            objects, message = find_objects(self.find_client, req.queries, req.color, self.find_attempts,
                                            self.search_min_score)
            # 確信度の高い順では、近くの別の物を返すことがあるため、追ってきた物に一番近いものを返す。
            obj = self.nearest(objects, tracker.target)
            if obj is None:
                raise Failed(ApproachObject.Result.NOT_FOUND,
                             f'lost after approaching: {message or "only other objects are in view"}')
            self.remember(obj)
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

    def drive_to(self, goal_handle, tracker, standoff, fb):
        """カメラを物体へ向け続けながら、車軸の中点から見た物体がstandoffの位置・正面に来るまで動く。"""
        if self.gaze is None:
            raise Failed(ApproachObject.Result.DRIVE_FAILED, 'robot_description not received yet')
        # 探す姿勢からカメラを物体へ向ける姿勢へは、車体やLiDARに当たらないようMoveItで移る。以降は少しずつ直接送る。
        gaze = self.aim(tracker.target, self.current_pose(), dict(zip(self.arm_joints, self.gaze_pose)))
        self.arm_to([gaze[n] for n in self.arm_joints])
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
            heading = math.atan2(ly, dx)
            remaining = math.hypot(dx, ly) - (standoff - self.rotation_center_x)
            goal_handle.publish_feedback(fb('driving', remaining))
            if time.time() >= next_gaze:
                gaze = self.aim(tracker.target, (x, y, yaw), gaze)
                self.stream_arm(gaze)
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
        """odom上の物体へカメラの光軸を向ける腕の関節角度(付け根と手首だけを動かす)。"""
        fixed = dict(zip(self.arm_joints, self.gaze_pose))
        q, _ = self.gaze.solve(self.to_base(target, pose), fixed, seed,
                               limits={'wrist_flex': tuple(self.gaze_wrist_limits)})
        return q

    def stream_arm(self, joints):
        # 前の目標は新しい目標で置き換わる(JTCが受け付けた時点で前の軌道を捨てる)ため、結果は待たない。
        t = 2.0 * self.gaze_period
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.arm_joints)
        goal.trajectory.points = [JointTrajectoryPoint(positions=[float(joints[n]) for n in self.arm_joints],
                                                       time_from_start=DurationMsg(sec=0, nanosec=int(t * 1e9)))]
        self.arm_client.send_goal_async(goal)

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

    def __init__(self, node, queries, color, target):
        self.node, self.queries, self.color = node, queries, color
        self.target = target
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
            best = None
            for obj in objects:
                p = self.node.to_odom(obj)
                # 別の同じような物に乗り換えないよう、今の推定の近くにある物だけを使う。
                if p is not None and math.dist(p[:2], self.target[:2]) <= self.node.track_gate:
                    if best is None or math.dist(p[:2], self.target[:2]) < math.dist(best[:2], self.target[:2]):
                        best = p
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
