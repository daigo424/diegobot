"""リーダーアームでのテレオペを、キーボードの合図でエピソードごとにrosbag(mcap)へ記録する。"""
import json
import os
import shutil
import sys
import termios
import threading
import time

import rclpy
import rosbag2_py
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from amr_teleop.arm_control import move_head

HELP = ('Enter: 記録を始める/止めて保存  x: 記録中なら捨てる・止めた後なら直前のエピソードを消す\n'
        'l: 保存したエピソードの一覧  d <番号>: そのエピソードを消す(次の記録はその番号から)  q: 終わる  (文字の後にEnter)')


class EpisodeRecorder(Node):
    def __init__(self):
        super().__init__('record_episodes')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.root = os.path.join(p('output_dir', '/data/vla/raw'), p('dataset', 'test'))
        self.task = p('task', '')
        self.min_duration = p('min_duration', 1.0)
        self.head_joints = p('head_joints', ['head_pan', 'head_tilt'])
        # 渡さなければ頭部は動かさず、記録を始めた時点の向きをそのまま使う。
        self.fixed_head_pose = self.declare_parameter('head_pose', Parameter.Type.DOUBLE_ARRAY).value
        self.head_pose = None
        self.current_head = None
        topics = p('topics', ['/head_camera/color/image_raw/compressed', '/camera/camera/color/image_raw/compressed',
                              '/joint_states', '/leader/joint_states'])
        types = p('types', ['sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/CompressedImage',
                            'sensor_msgs/msg/JointState', 'sensor_msgs/msg/JointState'])
        if not self.task:
            raise ValueError('task (the instruction text for this dataset) is required')
        os.makedirs(self.root, exist_ok=True)
        self.topics = list(zip(topics, types))
        self.lock = threading.Lock()
        self.writer = None
        self.episode_dir = None
        self.last_saved = None
        self.started = 0.0
        self.counts = {}
        self.head = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, qos_profile_sensor_data)
        for name, type_name in self.topics:
            msg_type = self._import(type_name)
            # 生のバイト列のまま受け取り、デシリアライズせずにbagへ書く。
            self.create_subscription(msg_type, name, lambda data, n=name: self.on_message(n, data),
                                     qos_profile_sensor_data, raw=True)

    @staticmethod
    def _import(type_name):
        pkg, _, cls = type_name.split('/')
        return getattr(__import__(f'{pkg}.msg', fromlist=[cls]), cls)

    def on_message(self, name, data):
        with self.lock:
            if self.writer is not None:
                self.writer.write(name, data, self.get_clock().now().nanoseconds)
                self.counts[name] = self.counts.get(name, 0) + 1

    def on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in self.head_joints):
            self.current_head = [round(pos[j], 3) for j in self.head_joints]

    def episodes(self):
        return sorted(int(d.split('_')[1]) for d in os.listdir(self.root) if d.startswith('episode_'))

    def path_of(self, index):
        return os.path.join(self.root, f'episode_{index:06d}')

    def next_episode_dir(self):
        # 消したエピソードの番号を先に埋める(記録の手順を番号で管理しているため、消した番号から撮り直せるようにする)。
        done = set(self.episodes())
        return self.path_of(next(i for i in range(len(done) + 1) if i not in done))

    def list_episodes(self):
        lines = []
        for i in self.episodes():
            info = os.path.join(self.path_of(i), 'episode.json')
            if os.path.exists(info):
                with open(info) as f:
                    d = json.load(f)
                lines.append(f'  {i:3d}: {d["duration"]:5.1f}秒  頭部{d.get("head_pose")}')
            else:
                lines.append(f'  {i:3d}: 保存されていない(記録の途中で終わった)。d {i} で消せる')
        print('\n'.join(lines) if lines else '  まだ無い', flush=True)
        self.print_next()

    def print_next(self):
        saved = sum(os.path.exists(os.path.join(self.path_of(i), 'episode.json')) for i in self.episodes())
        print(f'保存済み{saved}本、次は {os.path.basename(self.next_episode_dir())}', flush=True)

    def delete(self, arg):
        try:
            index = int(arg)
        except ValueError:
            print('使い方: d <番号>', flush=True)
            return
        path = self.path_of(index)
        if not os.path.isdir(path):
            print(f'{os.path.basename(path)} は無い', flush=True)
            return
        print(f'{os.path.basename(path)} を消す? 消すなら y+Enter', flush=True)
        if sys.stdin.readline().strip().lower() != 'y':
            print('消さなかった', flush=True)
            return
        shutil.rmtree(path, ignore_errors=True)
        if self.last_saved == path:
            self.last_saved = None
        print(f'消した: {os.path.basename(path)}', flush=True)
        self.print_next()

    def move_head(self):
        ok = move_head(self.head, self.head_joints, self.fixed_head_pose)
        if not ok:
            self.get_logger().error('could not move the head to the recording pose')
        return ok

    def start(self):
        if self.fixed_head_pose is not None:
            print('頭部を記録の向きに動かしている…', flush=True)
            if not self.move_head():
                return
            self.head_pose = list(self.fixed_head_pose)
        elif self.current_head is None:
            print('頭部の関節角が届いていないため記録を始めない', flush=True)
            return
        else:
            self.head_pose = list(self.current_head)
        path = self.next_episode_dir()
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
        for i, (name, type_name) in enumerate(self.topics):
            writer.create_topic(rosbag2_py.TopicMetadata(i, name, type_name, 'cdr'))
        with self.lock:
            self.counts = {}
            self.writer = writer
            self.episode_dir = path
            self.started = time.time()
        # 始まるのを待つ間に押されたEnterが「止めて保存」として読まれないよう捨てる。
        try:
            termios.tcflush(sys.stdin, termios.TCIFLUSH)
        except termios.error:
            pass
        print(f'記録中: {os.path.basename(path)} (頭部 パン{self.head_pose[0]:+.2f} チルト{self.head_pose[1]:+.2f})', flush=True)

    def stop(self, keep):
        with self.lock:
            writer, path, counts = self.writer, self.episode_dir, self.counts
            self.writer = None
        duration = time.time() - self.started
        del writer
        if not keep:
            shutil.rmtree(path, ignore_errors=True)
            print(f'捨てた: {os.path.basename(path)}', flush=True)
            self.print_next()
            return
        missing = [n for n, _ in self.topics if counts.get(n, 0) == 0]
        if missing or duration < self.min_duration:
            shutil.rmtree(path, ignore_errors=True)
            print(f'保存せずに捨てた: {os.path.basename(path)} {duration:.1f}秒 届かなかったトピック {missing}', flush=True)
            self.print_next()
            return
        with open(os.path.join(path, 'episode.json'), 'w') as f:
            json.dump({'task': self.task, 'head_pose': self.head_pose, 'duration': duration, 'counts': counts}, f, ensure_ascii=False, indent=1)
        self.last_saved = path
        rates = ' '.join(f'{n.split("/")[1]}:{counts.get(n, 0) / duration:.1f}Hz' for n, _ in self.topics)
        print(f'保存: {os.path.basename(path)} {duration:.1f}秒 {rates}', flush=True)
        self.print_next()

    def keyboard(self):
        print(f'保存先 {self.root}\n指示文「{self.task}」\n{HELP}', flush=True)
        self.print_next()
        while True:
            line = sys.stdin.readline()
            if not line:
                break
            key = line.strip().lower()
            recording = self.writer is not None
            if not recording and key == 'l':
                self.list_episodes()
                continue
            if not recording and key.startswith('d'):
                self.delete(key[1:].strip())
                continue
            if key == 'q':
                if recording:
                    self.stop(keep=False)
                break
            if key == 'x':
                if recording:
                    self.stop(keep=False)
                elif self.last_saved:
                    shutil.rmtree(self.last_saved, ignore_errors=True)
                    print(f'消した: {os.path.basename(self.last_saved)}', flush=True)
                    self.last_saved = None
                    self.print_next()
            elif key == '' and recording:
                self.stop(keep=True)
            elif key == '':
                self.start()
        rclpy.try_shutdown()


def main():
    rclpy.init()
    node = EpisodeRecorder()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=node.keyboard, daemon=True).start()
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.writer is not None:
            node.stop(keep=False)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
