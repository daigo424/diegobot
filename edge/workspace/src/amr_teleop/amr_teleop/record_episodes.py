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
from amr_interfaces.srv import FindObjects
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import JointState

from amr_teleop.arm_control import move_head

HELP = ('Enter: 記録を始める/止めて保存  f: 記録中なら止めて失敗として保存  x: 記録中なら捨てる・止めた後なら直前のエピソードを消す\n'
        'l: 保存したエピソードの一覧  d <番号>: そのエピソードを消す(次の記録はその番号から)  n <メモ>: 次からの場面のメモ\n'
        's <長さ> <幅> <高さ>: 次からの物の実測の寸法[cm](s だけで消す)\n'
        'q: 終わる  (文字の後にEnter)')
# 一度しか出ない(後から購読しても前の値が届く)トピック。最新を持っておき、エピソードの始めにbagへ書く。
LATCHED = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


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
        self.object = None
        self.object_image = None
        self.current_head = None
        topics = p('topics', ['/head_camera/color/image_raw/compressed', '/camera/camera/color/image_raw/compressed',
                              '/joint_states', '/leader/joint_states'])
        types = p('types', ['sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/CompressedImage',
                            'sensor_msgs/msg/JointState', 'sensor_msgs/msg/JointState'])
        # 届かなくても捨てない、学習以外(シミュレーターでの再現・解析)に使うトピック。
        extra_topics = p('extra_topics', ['/arm_teleop_controller/commands'])
        extra_types = p('extra_types', ['std_msgs/msg/Float64MultiArray'])
        # 最新の1件だけをエピソードの始めに書くトピック。カメラの内部パラメータと、始める時の頭部の深度。
        snapshot_topics = p('snapshot_topics', ['/head_camera/color/camera_info', '/camera/camera/color/camera_info',
                                                '/head_camera/depth/image_raw/compressed'])
        snapshot_types = p('snapshot_types', ['sensor_msgs/msg/CameraInfo', 'sensor_msgs/msg/CameraInfo',
                                              'sensor_msgs/msg/CompressedImage'])
        # 物の位置を測る対象(英語の名詞句、例: "blue block")。空なら測らない。
        self.object_queries = [q for q in p('object', '').split(',') if q.strip()]
        self.find_service = p('find_service', '/head_object_finder/find')
        self.find_settle = p('find_settle', 1.0)
        self.scene = p('scene', '')
        self.object_size = None
        self.head_color_topic = topics[0]
        self.last_head_color = None
        if not self.task:
            raise ValueError('task (the instruction text for this dataset) is required')
        os.makedirs(self.root, exist_ok=True)
        self.failed_root = self.root + '_failed'
        self.topics = list(zip(topics, types))
        self.extra_topics = list(zip(extra_topics, extra_types))
        self.snapshot_topics = list(zip(snapshot_topics, snapshot_types))
        self.snapshots = {}
        self.tf_static = []
        self.find = self.create_client(FindObjects, self.find_service)
        self.lock = threading.Lock()
        self.writer = None
        self.episode_dir = None
        self.last_saved = None
        self.started = 0.0
        self.counts = {}
        self.head = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, qos_profile_sensor_data)
        for name, type_name in self.topics + self.extra_topics:
            msg_type = self._import(type_name)
            # 生のバイト列のまま受け取り、デシリアライズせずにbagへ書く。
            self.create_subscription(msg_type, name, lambda data, n=name: self.on_message(n, data),
                                     qos_profile_sensor_data, raw=True)
        for name, type_name in self.snapshot_topics:
            self.create_subscription(self._import(type_name), name,
                                     lambda data, n=name: self.snapshots.__setitem__(n, data),
                                     qos_profile_sensor_data, raw=True)
        # 静的なTFは送り手ごとに1件ずつ届くため、全部持っておく。
        self.create_subscription(self._import('tf2_msgs/msg/TFMessage'), '/tf_static',
                                 lambda data: self.tf_static.append(data), LATCHED, raw=True)

    @staticmethod
    def _import(type_name):
        pkg, _, cls = type_name.split('/')
        return getattr(__import__(f'{pkg}.msg', fromlist=[cls]), cls)

    def on_message(self, name, data):
        if name == self.head_color_topic:
            self.last_head_color = data
        with self.lock:
            if self.writer is not None:
                self.writer.write(name, data, self.get_clock().now().nanoseconds)
                self.counts[name] = self.counts.get(name, 0) + 1

    def on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in self.head_joints):
            self.current_head = [round(pos[j], 3) for j in self.head_joints]

    def episodes(self, root=None):
        root = root or self.root
        if not os.path.isdir(root):
            return []
        return sorted(int(d.split('_')[1]) for d in os.listdir(root) if d.startswith('episode_'))

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

    def measure_object(self):
        """頭部カメラで物の位置・向き・大きさを測る。腕が動く前の、置いた時の姿勢をシミュレーターで再現するため。"""
        self.object_image = None
        if not self.object_queries:
            return None
        if not self.find.wait_for_service(timeout_sec=1.0):
            print(f'{self.find_service} が無いため物の位置を測らない(make perceptionを起動すると測る)', flush=True)
            return {'queries': self.object_queries, 'error': 'service unavailable'}
        # 頭部が着いた直後の画像は揺れていることがあるため、少し待ってから撮った画像で測る。
        time.sleep(self.find_settle)
        done = threading.Event()
        future = self.find.call_async(FindObjects.Request(queries=self.object_queries, want_obb=True, want_mask=True))
        future.add_done_callback(lambda f: done.set())
        if not done.wait(5.0) or future.result() is None:
            print('物の位置を測れなかった(応答が無い)', flush=True)
            return {'queries': self.object_queries, 'error': 'no response'}
        res = future.result()
        objects = []
        for o in res.objects:
            q = o.obb_pose.pose.orientation
            objects.append({
                'bbox_px': [round(o.bbox.center.position.x), round(o.bbox.center.position.y),
                            round(o.bbox.size_x), round(o.bbox.size_y)],
                'label': o.label, 'score': round(o.score, 3), 'color': o.color,
                'position': [round(v, 4) for v in (o.position.point.x, o.position.point.y, o.position.point.z)]
                if o.position_valid else None,
                'obb_valid': o.obb_valid,
                'obb_position': [round(v, 4) for v in (o.obb_pose.pose.position.x, o.obb_pose.pose.position.y,
                                                       o.obb_pose.pose.position.z)],
                'obb_quaternion_xyzw': [round(v, 5) for v in (q.x, q.y, q.z, q.w)],
                'obb_extents': [round(v, 4) for v in (o.obb_extents.x, o.obb_extents.y, o.obb_extents.z)],
            })
        best = max(objects, key=lambda o: o['score'], default=None)
        if best:
            pos = best['obb_position'] if best['obb_valid'] else best['position']
            print(f'物: {best["label"]} {best["color"]} 確信度{best["score"]:.2f} 位置{pos}', flush=True)
        else:
            print(f'物が見つからなかった({res.message})', flush=True)
        self.object_image = bytes(res.image.data) if res.image.data else None
        return {'queries': self.object_queries, 'frame': 'base_link', 'message': res.message, 'objects': objects}

    def save_start_images(self, path):
        """始める時の頭部の画像と、測った物を囲んだ画像・切り出した画像。後で物を見て寸法を測れるように。"""
        image = self.object_image
        if image is None and self.last_head_color is not None:
            image = bytes(deserialize_message(self.last_head_color, CompressedImage).data)
        if image is None:
            return
        with open(os.path.join(path, 'start_head.jpg'), 'wb') as f:
            f.write(image)
        objects = (self.object or {}).get('objects') or []
        if not objects or self.object_image is None:
            return
        try:
            import cv2
            import numpy as np
        except ImportError:
            return
        frame = cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        boxed = frame.copy()
        for i, o in enumerate(sorted(objects, key=lambda o: -o['score'])):
            cx, cy, w, h = o['bbox_px']
            x0, y0, x1, y1 = cx - w // 2, cy - h // 2, cx + w // 2, cy + h // 2
            color = (0, 0, 255) if i == 0 else (0, 200, 255)
            cv2.rectangle(boxed, (x0, y0), (x1, y1), color, 2)
            cv2.putText(boxed, f'{o["label"]} {o["score"]:.2f}', (x0, max(12, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        color, 1)
            if i == 0:
                # 切り出しは周りも少し入れ、物の形と大きさの見当が付くようにする。
                m = max(w, h) // 2 + 20
                crop = frame[max(0, cy - m):cy + m, max(0, cx - m):cx + m]
                if crop.size:
                    cv2.imwrite(os.path.join(path, 'object_crop.jpg'), cv2.resize(crop, None, fx=3, fy=3,
                                                                              interpolation=cv2.INTER_CUBIC))
        cv2.imwrite(os.path.join(path, 'object_head.jpg'), boxed)

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
        self.object = self.measure_object()
        path = self.next_episode_dir()
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
        all_topics = self.topics + self.extra_topics + self.snapshot_topics + [('/tf_static', 'tf2_msgs/msg/TFMessage')]
        for i, (name, type_name) in enumerate(all_topics):
            writer.create_topic(rosbag2_py.TopicMetadata(i, name, type_name, 'cdr'))
        now = self.get_clock().now().nanoseconds
        for name, data in list(self.snapshots.items()):
            writer.write(name, data, now)
        for data in list(self.tf_static):
            writer.write('/tf_static', data, now)
        self.save_start_images(path)
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

    def stop(self, keep, success=True):
        with self.lock:
            writer, path, counts = self.writer, self.episode_dir, self.counts
            self.writer = None
        duration = time.time() - self.started
        del writer
        if keep and not success:
            # 失敗は学習に使わないため別のデータセットへ移し、記録の手順で使う番号を空けておく。
            os.makedirs(self.failed_root, exist_ok=True)
            done = set(self.episodes(self.failed_root))
            failed = os.path.join(self.failed_root, f'episode_{next(i for i in range(len(done) + 1) if i not in done):06d}')
            shutil.move(path, failed)
            path = failed
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
            json.dump({'task': self.task, 'success': success, 'head_pose': self.head_pose, 'duration': duration,
                       'counts': counts, 'object': self.object, 'object_size_cm': self.object_size, 'scene': self.scene,
                       'snapshots': sorted(self.snapshots) + (['/tf_static'] if self.tf_static else [])},
                      f, ensure_ascii=False, indent=1)
        self.last_saved = path
        rates = ' '.join(f'{n.split("/")[1]}:{counts.get(n, 0) / duration:.1f}Hz' for n, _ in self.topics)
        label = '保存' if success else f'失敗として保存({os.path.basename(self.failed_root)})'
        print(f'{label}: {os.path.basename(path)} {duration:.1f}秒 {rates}', flush=True)
        missing_snapshots = [n for n, _ in self.snapshot_topics if n not in self.snapshots]
        if missing_snapshots or not self.tf_static:
            print(f'  届いていない: {missing_snapshots + ([] if self.tf_static else ["/tf_static"])}', flush=True)
        self.print_next()

    def keyboard(self):
        print(f'保存先 {self.root}\n指示文「{self.task}」 測る物 {self.object_queries or "(測らない)"} 場面のメモ「{self.scene}」\n{HELP}', flush=True)
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
            if key.startswith('s ') or key == 's':
                try:
                    size = [float(v) for v in key[1:].split()]
                except ValueError:
                    size = []
                if key == 's':
                    self.object_size = None
                    print('物の寸法を消した', flush=True)
                elif len(size) == 3:
                    self.object_size = size
                    print(f'物の寸法: {size[0]} × {size[1]} × {size[2]} cm', flush=True)
                else:
                    print('使い方: s <長さ> <幅> <高さ> (cm)', flush=True)
                continue
            if key.startswith('n ') or key == 'n':
                self.scene = line.strip()[1:].strip()
                print(f'場面のメモ: 「{self.scene}」', flush=True)
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
            elif key == 'f' and recording:
                self.stop(keep=True, success=False)
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
