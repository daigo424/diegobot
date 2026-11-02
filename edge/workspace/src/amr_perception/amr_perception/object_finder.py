import os
from collections import deque

import cv2
import numpy as np
import rclpy
from amr_interfaces.msg import DetectedObject
from amr_interfaces.srv import FindObjects
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLOE
from visualization_msgs.msg import Marker, MarkerArray
import tf2_geometry_msgs  # noqa: F401  PointStampedをBuffer.transformで変換できるよう登録する

# OpenCVのHSVの色相(0〜179)の区間ごとの色名。赤は0付近と179付近にまたがる。
HUE_NAMES = [(5, 'red'), (18, 'orange'), (33, 'yellow'), (80, 'green'), (95, 'cyan'), (130, 'blue'),
             (150, 'purple'), (170, 'pink'), (180, 'red')]


def color_name(bgr_pixels):
    if len(bgr_pixels) == 0:
        return ''
    hsv =cv2.cvtColor(bgr_pixels.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)
    h, s, v = hsv[:, 0], hsv[:, 1], hsv[:, 2]
    names = np.full(len(hsv), 'gray', dtype=object)
    names[v < 50] = 'black'
    names[(v >= 50) & (s < 40) & (v > 180)] = 'white'
    chromatic = (v >= 50) & (s >= 40)
    lower = 0
    for upper, name in HUE_NAMES:
        names[chromatic & (h >= lower) & (h < upper)] = name
        lower = upper
    values, counts = np.unique(names, return_counts=True)
    return str(values[np.argmax(counts)])


def decode_depth(msg):
    depth = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)
    if depth is None:
        return None
    # compressedトランスポートのPNGは16UC1[mm]のまま来る。
    return depth.astype(np.float32) / 1000.0 if depth.dtype == np.uint16 else depth.astype(np.float32)


class ObjectFinder(Node):
    def __init__(self):
        super().__init__('object_finder')
        models_dir = self.declare_parameter('models_dir', '/data/models').value
        # 探す対象を呼び出しごとに文字で変えるため、学習済み語彙固定の-pf無しのモデルを使う。
        # sだと真上から見た積み木などの確信度が0.25前後で揺れて見落とすため、mにする。
        model_path = self.declare_parameter('model', 'yoloe-26m-seg.pt').value
        self.conf = self.declare_parameter('conf', 0.25).value
        self.target_frame = self.declare_parameter('target_frame', 'base_link').value
        self.max_image_age = self.declare_parameter('max_image_age', 1.0).value

        os.makedirs(models_dir, exist_ok=True)
        os.chdir(models_dir)
        self.model = YOLOE(model_path)
        self.classes = None

        self.tf_buffer = Buffer()
        # サービス処理中にTFを待つため、TFの受信は専用ノード(node=None)を別スレッドで回す。
        # selfを渡すとrclpy.spinのexecutorに取られ、処理中はTFが更新されず変換に失敗する。
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)

        self.camera_info = None
        # カラーと深度は同じ時刻のフレームを組にする。別々に届き、最新のカラーの深度がまだ無いことがあるため両方溜めておく。
        self.colors = deque(maxlen=10)
        self.depths = deque(maxlen=10)
        self.create_subscription(CompressedImage, 'color/image/compressed', self.on_color, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, 'depth/image/compressed', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'color/camera_info', self.on_camera_info, qos_profile_sensor_data)

        self.image_pub = self.create_publisher(Image, '~/image', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', 1)
        self.create_service(FindObjects, '~/find', self.on_find)

    def on_color(self, msg):
        self.colors.append(msg)

    def on_depth(self, msg):
        self.depths.append(msg)

    def on_camera_info(self, msg):
        self.camera_info = msg

    def matching_depth(self, stamp):
        target = Time.from_msg(stamp)
        best = min(self.depths, key=lambda d: abs((Time.from_msg(d.header.stamp) - target).nanoseconds), default=None)
        if best is None or abs((Time.from_msg(best.header.stamp) - target).nanoseconds) > 20_000_000:
            return None
        return best

    def on_find(self, request, response):
        queries = [q.strip() for q in request.queries if q.strip()]
        if not queries:
            response.message = 'queries is empty'
            return response
        info = self.camera_info
        if not self.colors or info is None:
            response.message = 'no color image or camera_info received yet'
            return response
        pairs = ((c, self.matching_depth(c.header.stamp)) for c in reversed(list(self.colors)))
        color_msg, depth_msg = next(((c, d) for c, d in pairs if d is not None), (None, None))
        if color_msg is None:
            response.message = 'no depth image matching the color images'
            return response
        age = self.get_clock().now() - Time.from_msg(color_msg.header.stamp)
        if age > Duration(seconds=self.max_image_age):
            response.message = f'latest color image with depth is {age.nanoseconds / 1e9:.1f}s old'
            return response

        frame = cv2.imdecode(np.frombuffer(color_msg.data, np.uint8), cv2.IMREAD_COLOR)
        depth = decode_depth(depth_msg)
        if frame is None or depth is None or depth.shape[:2] != frame.shape[:2]:
            response.message = 'failed to decode images or their sizes differ'
            return response

        # 文字指定の埋め込み計算は重いため、前回と同じ対象なら作り直さない。
        if queries != self.classes:
            self.model.set_classes(queries)
            self.classes = queries
        # 深度と画素単位で重ねるため、マスクを入力画像の解像度で受け取る。
        result = self.model.predict(frame, conf=self.conf, retina_masks=True, verbose=False)[0]

        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        masks = result.masks.data.cpu().numpy().astype(bool) if result.masks is not None else []
        for box, conf, cls, mask in zip(result.boxes.xywh.tolist(), result.boxes.conf.tolist(),
                                        result.boxes.cls.tolist(), masks):
            obj = DetectedObject(label=result.names[int(cls)], score=conf, color=color_name(frame[mask]))
            obj.bbox.center.position.x, obj.bbox.center.position.y = box[0], box[1]
            obj.bbox.size_x, obj.bbox.size_y = box[2], box[3]

            valid = mask & (depth > 0) & np.isfinite(depth)
            if valid.any():
                vs, us = np.nonzero(valid)
                z = float(np.median(depth[valid]))
                point = PointStamped()
                point.header.stamp = color_msg.header.stamp
                point.header.frame_id = info.header.frame_id
                point.point.x = (float(us.mean()) - cx) * z / fx
                point.point.y = (float(vs.mean()) - cy) * z / fy
                point.point.z = z
                try:
                    obj.position = self.tf_buffer.transform(point, self.target_frame, timeout=Duration(seconds=0.2))
                    obj.position_valid = True
                    obj.width, obj.height = box[2] * z / fx, box[3] * z / fy
                except TransformException as e:
                    self.get_logger().warn(f'cannot transform to {self.target_frame}: {e}')
            response.objects.append(obj)

        response.success = True
        response.message = f'found {len(response.objects)} object(s)'
        self.publish_debug(color_msg.header, result, response.objects)
        return response

    def publish_debug(self, header, result, objects):
        if self.image_pub.get_subscription_count() > 0:
            annotated = result.plot()
            self.image_pub.publish(Image(header=header, height=annotated.shape[0], width=annotated.shape[1],
                                         encoding='bgr8', step=annotated.shape[1] * 3, data=annotated.tobytes()))
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for i, obj in enumerate(o for o in objects if o.position_valid):
            # rclpyのメッセージ代入はコピーされないため、値をコピーして返却値の位置を書き換えないようにする。
            p = obj.position.point
            sphere = Marker(header=obj.position.header, ns='objects', id=i, type=Marker.SPHERE, action=Marker.ADD)
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = p.x, p.y, p.z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.03
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.8, 0.2, 1.0, 1.0
            # RVizは空白の幅を文字の大きさに合わせて縮めず単語が大きく離れて表示されるため、空白を使わない。
            text = f'{obj.label}/{obj.color}/{obj.score:.2f}'.replace(' ', '_')
            label = Marker(header=obj.position.header, ns='labels', id=i, type=Marker.TEXT_VIEW_FACING,
                           action=Marker.ADD, text=text)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = p.x, p.y, p.z + 0.05
            label.pose.orientation.w = 1.0
            label.scale.z = 0.03
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            markers.markers += [sphere, label]
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = ObjectFinder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
