import os

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from ultralytics import YOLOE
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        models_dir = self.declare_parameter('models_dir', '/data/models').value
        model_path = self.declare_parameter('model', 'yoloe-26s-seg-pf.pt').value
        # 未指定なら学習済みの語彙すべてを対象にする(-pf.ptのモデル用)。指定する場合は-pf無しの-seg.ptを使う。
        # TensorRTエンジン(.engine)は書き出し時に対象が焼き込まれるため指定できない(export_engineで作り直す)。
        classes = self.declare_parameter('classes', Parameter.Type.STRING_ARRAY).value or []
        if classes and model_path.endswith('.engine'):
            raise ValueError('classes cannot be set for a TensorRT engine; re-export it with export_engine instead')
        self.conf = self.declare_parameter('conf', 0.25).value

        # Ultralyticsはモデルの重みや文字指定用のテキストエンコーダを作業ディレクトリへ自動ダウンロードするため、
        # 起動した場所にかかわらず置き場を固定する。
        os.makedirs(models_dir, exist_ok=True)
        os.chdir(models_dir)
        self.model = YOLOE(model_path)
        if classes:
            self.model.set_classes(classes)

        self.detections_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.image_pub = self.create_publisher(Image, 'detections/image', 1)
        # 推論が追いつかない間に溜まった古いフレームを処理しないよう、最新の1枚だけを保持する。
        self.create_subscription(CompressedImage, 'image/compressed', self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn(f'failed to decode image (format: {msg.format})', throttle_duration_sec=5.0)
            return
        result = self.model.predict(frame, conf=self.conf, verbose=False)[0]

        detections = Detection2DArray(header=msg.header)
        for xywh, conf, cls in zip(result.boxes.xywh.tolist(), result.boxes.conf.tolist(), result.boxes.cls.tolist()):
            detection = Detection2D(header=msg.header)
            detection.bbox.center.position.x, detection.bbox.center.position.y = xywh[0], xywh[1]
            detection.bbox.size_x, detection.bbox.size_y = xywh[2], xywh[3]
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = result.names[int(cls)]
            hypothesis.hypothesis.score = conf
            detection.results.append(hypothesis)
            detections.detections.append(detection)
        self.detections_pub.publish(detections)

        if self.image_pub.get_subscription_count() > 0:
            annotated = result.plot()
            self.image_pub.publish(Image(
                header=msg.header,
                height=annotated.shape[0],
                width=annotated.shape[1],
                encoding='bgr8',
                step=annotated.shape[1] * 3,
                data=annotated.tobytes(),
            ))


def main():
    rclpy.init()
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
