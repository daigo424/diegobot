"""リーダーアーム(SO-101)の関節角を読んで出し、実機のアームを追従させる。"""
import math

import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header, String

from amr_teleop.arm_control import joint_limits, switch_request
from amr_teleop.feetech_bus import FeetechBus

STEP_RAD = 2 * math.pi / 4096


class LeaderTeleop(Node):
    def __init__(self):
        super().__init__('leader_teleop')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.joints = p('joints', ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'])
        self.ids = p('servo_ids', [1, 2, 3, 4, 5, 6])
        self.zero_raw = p('zero_raw', [2048] * 6)
        self.zero_angle = p('zero_angle', [0.0] * 6)
        self.rate = p('rate', 50.0)
        self.sync_velocity = p('sync_velocity', 0.5)
        self.max_velocity = p('max_velocity', 3.0)
        self.sync_tolerance = p('sync_tolerance', 0.05)
        self.stale_timeout = p('stale_timeout', 0.2)
        self.follow = p('follow', True)
        self.teleop_controller = p('teleop_controller', 'arm_teleop_controller')
        self.normal_controllers = p('normal_controllers', ['arm_group_controller', 'grip_group_controller'])
        self.bus = FeetechBus(p('port', '/dev/ttyLeaderArm'))

        self.limits = None
        self.follower = None
        self.follower_time = None
        self.cmd = None
        self.synced = False
        self.switched = False
        self.switching = None
        self.last_read = self.get_clock().now()
        self.leader_pub = self.create_publisher(JointState, 'leader/joint_states', 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, f'/{self.teleop_controller}/commands', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_subscription(String, '/robot_description', self.on_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.create_timer(1.0 / self.rate, self.tick)

    def on_description(self, msg):
        self.limits = joint_limits(msg.data, self.joints)

    def on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in self.joints):
            self.follower = [pos[n] for n in self.joints]
            self.follower_time = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        try:
            raw = self.bus.read_positions(self.ids)
        except OSError as e:
            # USBが抜けるとI/Oエラーになる。ノードを落とさず、読めない時と同じく実機をその場に止めておく。
            self.get_logger().error(f'leader arm port error: {e}', throttle_duration_sec=2.0)
            raw = None
        if raw is None:
            if (now - self.last_read).nanoseconds * 1e-9 > self.stale_timeout:
                self.get_logger().warn('leader arm is not responding; holding the follower', throttle_duration_sec=1.0)
            return
        self.last_read = now
        leader = [(r - z) * STEP_RAD + a for r, z, a in zip(raw, self.zero_raw, self.zero_angle)]
        self.leader_pub.publish(JointState(header=Header(stamp=now.to_msg()), name=self.joints, position=leader))
        if not self.follow:
            return
        if self.limits is None or self.follower is None:
            self.get_logger().warn('waiting for /robot_description and the arm joint states', throttle_duration_sec=2.0)
            return
        if not self.switched:
            self.start_follow()
            return
        if (now - self.follower_time).nanoseconds * 1e-9 > self.stale_timeout * 5:
            # 実機のドライバが止まると関節角が届かなくなる。そのまま指令を送り続けても動かないため追従をやめる。
            self.follow = False
            self.get_logger().error('follower joint states stopped; stopped following')
            return
        target = [min(max(q, lo), hi) for q, (lo, hi) in zip(leader, self.limits)]
        # 追従を始めた時点でリーダーと実機の姿勢が違っていても跳ばないよう、最初は遅く寄せる。
        v = self.max_velocity if self.synced else self.sync_velocity
        step = v / self.rate
        self.cmd = [c + min(max(t - c, -step), step) for c, t in zip(self.cmd, target)]
        if not self.synced and all(abs(t - c) < self.sync_tolerance for c, t in zip(self.cmd, target)):
            self.synced = True
            self.get_logger().info('follower caught up with the leader; following')
        self.cmd_pub.publish(Float64MultiArray(data=self.cmd))

    def start_follow(self):
        # 切り替え中にタイマーの中で応答を待つとこのノードの処理が止まるため、応答は次の周期以降に見る。
        if self.switching is None:
            if not self.switch.service_is_ready():
                self.get_logger().info('waiting for the controller manager', throttle_duration_sec=2.0)
                return
            self.switching = self.switch.call_async(switch_request([self.teleop_controller], self.normal_controllers))
            return
        if not self.switching.done():
            return
        if self.switching.result() is not None and self.switching.result().ok:
            self.switched = True
            self.cmd = list(self.follower)
            self.get_logger().info('moving the follower to the leader pose')
        else:
            self.follow = False
            self.get_logger().error(f'could not activate {self.teleop_controller}; only publishing the leader')

    def stop_follow(self):
        if not self.switched:
            return
        self.get_logger().info(f'switching back to {self.normal_controllers}')
        future = self.switch.call_async(switch_request(self.normal_controllers, [self.teleop_controller]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None or not future.result().ok:
            self.get_logger().error(f'could not switch back from {self.teleop_controller}; make teleop runs check_arm to fix it')
        else:
            self.get_logger().info('switched back')



def main():
    # Ctrl+Cの後もコントローラを元に戻す呼び出しができるよう、rclpyにSIGINTでcontextを閉じさせない。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LeaderTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_follow()
        node.bus.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
