"""学習したポリシーで実機のアームを動かす。推論はROSの外のpolicy_server(開発機のGPU)にTCPで頼む。"""
import json
import socket
import struct
import sys
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray, String

from amr_teleop.arm_control import joint_limits, move_head, switch_request

HELP = ('Enter: 動かし始める/止める(その場で保持)  o: グリッパーを開く  h: グリッパーを開いて始める姿勢へ戻す  '
        'q: 終わる(コントローラを元に戻す)  (文字の後にEnter)')


class PolicyClient:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5.0)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def _recv_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError('policy server closed the connection')
            buf += chunk
        return bytes(buf)

    def request(self, header, images=None):
        images = images or {}
        header = dict(header, images={k: len(v) for k, v in images.items()})
        data = json.dumps(header).encode()
        self.sock.sendall(struct.pack('>I', len(data)) + data + b''.join(images.values()))
        return json.loads(self._recv_exact(struct.unpack('>I', self._recv_exact(4))[0]))


class PolicyRunner(Node):
    def __init__(self):
        super().__init__('policy_runner')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.joints = p('joints', ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'])
        self.camera_names = p('camera_names', ['head', 'wrist'])
        camera_topics = p('camera_topics', ['/head_camera/color/image_raw/compressed', '/camera/camera/color/image_raw/compressed'])
        self.head_joints = p('head_joints', ['head_pan', 'head_tilt'])
        self.head_pose = p('head_pose', [0.0, -0.78])
        self.task = p('task', 'Pick up the block')
        # 記録を変換した時のfpsと同じ周期で回す(ポリシーは1コマごとの動作を学んでいる)。
        self.rate = p('rate', 15.0)
        self.max_velocity = p('max_velocity', 2.0)
        self.stale_timeout = p('stale_timeout', 0.5)
        self.max_duration = p('max_duration', 30.0)
        self.gripper_open = p('gripper_open', 1.0)
        self.move_velocity = p('move_velocity', 1.0)
        # 記録で各エピソードを始めた姿勢に近い。ポリシーはここから動き始める場面しか学んでいない。
        self.home_pose = p('home_pose', [0.0, -1.6, 1.33, 1.39, -1.46, 0.0])
        self.teleop_controller = p('teleop_controller', 'arm_teleop_controller')
        self.normal_controllers = p('normal_controllers', ['arm_group_controller', 'grip_group_controller'])
        self.client = PolicyClient(p('host', '127.0.0.1'), p('port', 5555))

        self.lock = threading.Lock()
        self.images = {}
        self.follower = None
        self.follower_time = 0.0
        self.limits = None
        self.running = False
        self.switched = False
        self.cmd = None
        for name, topic in zip(self.camera_names, camera_topics):
            self.create_subscription(CompressedImage, topic, lambda msg, n=name: self.on_image(n, msg), qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, qos_profile_sensor_data)
        self.create_subscription(String, '/robot_description', self.on_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.cmd_pub = self.create_publisher(Float64MultiArray, f'/{self.teleop_controller}/commands', 10)
        self.switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.head = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

    def on_image(self, name, msg):
        with self.lock:
            self.images[name] = (time.monotonic(), bytes(msg.data))

    def on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in self.joints):
            with self.lock:
                self.follower = [pos[n] for n in self.joints]
                self.follower_time = time.monotonic()

    def on_description(self, msg):
        self.limits = joint_limits(msg.data, self.joints)

    def call_switch(self, activate, deactivate):
        if not self.switch.wait_for_service(timeout_sec=3.0):
            return False
        done = threading.Event()
        future = self.switch.call_async(switch_request(activate, deactivate))
        future.add_done_callback(lambda _: done.set())
        done.wait(5.0)
        return future.done() and future.result() is not None and future.result().ok

    def observation(self):
        now = time.monotonic()
        with self.lock:
            if self.follower is None or now - self.follower_time > self.stale_timeout:
                return None, 'joint states'
            for name in self.camera_names:
                if name not in self.images or now - self.images[name][0] > self.stale_timeout:
                    return None, f'{name} camera'
            return (list(self.follower), {n: self.images[n][1] for n in self.camera_names}), None

    def start(self):
        if self.limits is None:
            print('/robot_description が届いていない', flush=True)
            return
        if self.teleop_running():
            return
        print('頭部を記録の向きに動かしている…', flush=True)
        if not move_head(self.head, self.head_joints, self.head_pose):
            print('頭部を動かせなかった', flush=True)
            return
        obs, missing = self.observation()
        if obs is None:
            print(f'{missing} が届いていないため始めない', flush=True)
            return
        if not self.switched:
            if not self.call_switch([self.teleop_controller], self.normal_controllers):
                print(f'{self.teleop_controller} に切り替えられない', flush=True)
                return
            self.switched = True
        self.client.request({'reset': True})
        self.cmd = list(obs[0])
        print(f'動かしている(最長{self.max_duration:.0f}秒)', flush=True)
        self.run_loop(self.tick, self.max_velocity)

    def run_loop(self, tick, velocity):
        self.running = True
        threading.Thread(target=self.control_loop, args=(tick, velocity), daemon=True).start()

    def control_loop(self, tick, velocity):
        self.step = velocity / self.rate
        period = 1.0 / self.rate
        started = time.monotonic()
        next_tick = started
        while self.running:
            try:
                if not tick(started):
                    break
            except (OSError, ConnectionError, KeyError) as e:
                print(f'推論サーバーとのやり取りに失敗したので止めた: {e}', flush=True)
                break
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
        self.running = False
        print(f'止めた(その場で保持)。{HELP}', flush=True)

    def tick(self, started):
        if time.monotonic() - started > self.max_duration:
            print('最長の時間に達したので止めた', flush=True)
            return False
        obs, missing = self.observation()
        if obs is None:
            # 観測が途切れたまま古い画像で動かし続けないよう、その場で止める。
            print(f'{missing} が途切れたので止めた', flush=True)
            return False
        state, images = obs
        reply = self.client.request({'state': state, 'task': self.task}, images)
        self.step_toward(reply['action'])
        return True

    def step_toward(self, target):
        target = [min(max(a, lo), hi) for a, (lo, hi) in zip(target, self.limits)]
        self.cmd = [c + min(max(t - c, -self.step), self.step) for c, t in zip(self.cmd, target)]
        self.cmd_pub.publish(Float64MultiArray(data=self.cmd))

    def teleop_running(self):
        topic = f'/{self.teleop_controller}/commands'
        if self.count_publishers(topic) > 1:
            # テレオペ(make teleop)が動いたままだと、リーダーとこのノードの指令がぶつかる。
            print(f'{topic} にほかのノードも指令を出している。make teleop を止めてから使ってください', flush=True)
            return True
        return False

    def prepare_move(self):
        if self.running:
            print('Enterでポリシーを止めてから使ってください', flush=True)
            return False
        if self.limits is None:
            print('/robot_description が届いていない', flush=True)
            return False
        if self.teleop_running():
            return False
        if not self.switched:
            obs, missing = self.observation()
            if obs is None:
                print(f'{missing} が届いていないため動かさない', flush=True)
                return False
            if not self.call_switch([self.teleop_controller], self.normal_controllers):
                print(f'{self.teleop_controller} に切り替えられない', flush=True)
                return False
            self.switched = True
            self.cmd = list(obs[0])
        return True

    def move_through(self, targets, label):
        print(f'{label}(Enterで止める)', flush=True)
        targets = list(targets)

        def tick(_):
            # 関節ごとに別々の速さで寄せると途中で変な姿勢を通るため、全関節が同時に着くよう比例させる。
            target = [min(max(a, lo), hi) for a, (lo, hi) in zip(targets[0], self.limits)]
            diff = [t - c for c, t in zip(self.cmd, target)]
            scale = min(1.0, self.step / max(max(abs(d) for d in diff), 1e-9))
            self.cmd = [c + d * scale for c, d in zip(self.cmd, diff)]
            self.cmd_pub.publish(Float64MultiArray(data=self.cmd))
            if scale == 1.0:
                targets.pop(0)
            return bool(targets)
        self.run_loop(tick, self.move_velocity)

    def with_gripper(self, pose, value):
        i = self.joints.index('gripper')
        return pose[:i] + [value] + pose[i + 1:]

    def open_gripper(self):
        if self.prepare_move():
            self.move_through([self.with_gripper(self.cmd, self.gripper_open)], 'グリッパーを開いている')

    def go_home(self):
        if self.prepare_move():
            opened = self.gripper_open
            self.move_through([self.with_gripper(self.cmd, opened), self.with_gripper(self.home_pose, opened), self.home_pose],
                              'グリッパーを開いて、始める姿勢へ戻している')

    def stop(self):
        self.running = False

    def restore(self):
        self.running = False
        if not self.switched:
            return
        if self.call_switch(self.normal_controllers, [self.teleop_controller]):
            self.switched = False
        else:
            self.get_logger().error(f'could not switch back from {self.teleop_controller}')

    def keyboard(self):
        print(HELP, flush=True)
        for line in sys.stdin:
            key = line.strip().lower()
            if key == 'q':
                break
            if key == 'o':
                self.open_gripper()
            elif key == 'h':
                self.go_home()
            if key == '':
                if self.running:
                    self.stop()
                else:
                    self.start()
        self.restore()
        rclpy.try_shutdown()


def main():
    # Ctrl+Cの後もコントローラを元に戻す呼び出しができるよう、rclpyにSIGINTでcontextを閉じさせない。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PolicyRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=node.keyboard, daemon=True).start()
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            # 元に戻す切り替えの応答を受け取るため、もう一度executorを回す。
            threading.Thread(target=executor.spin, daemon=True).start()
            node.restore()
            executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
