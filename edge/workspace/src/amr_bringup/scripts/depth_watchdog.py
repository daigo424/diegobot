#!/usr/bin/env python3
"""深度画像のうち値のある画素の割合を見張り、低いままなら終了して、launchごとカメラを開き直させる。"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthWatchdog(Node):
    def __init__(self):
        super().__init__('depth_watchdog')
        self.min_valid_ratio = self.declare_parameter('min_valid_ratio', 0.7).value
        self.bad_duration = self.declare_parameter('bad_duration', 10.0).value
        # 開き直した直後は、深度が揃うまで十数秒かかる。
        self.grace = self.declare_parameter('grace', 30.0).value
        self.check_period = self.declare_parameter('check_period', 1.0).value
        self.started = time.monotonic()
        self.bad_since = None
        self.next_check = 0.0
        self.failed = False
        self.create_subscription(Image, 'depth/image_raw', self.on_depth, qos_profile_sensor_data)

    def on_depth(self, msg):
        now = time.monotonic()
        if now < self.next_check or now - self.started < self.grace:
            return
        self.next_check = now + self.check_period
        ratio = float(np.count_nonzero(np.frombuffer(msg.data, np.uint16))) / (msg.width * msg.height)
        if ratio >= self.min_valid_ratio:
            if self.bad_since is not None:
                self.get_logger().info(f'depth recovered ({ratio:.0%} valid)')
            self.bad_since = None
            return
        if self.bad_since is None:
            self.bad_since = now
            self.get_logger().warn(f'only {ratio:.0%} of depth pixels are valid')
        elif now - self.bad_since >= self.bad_duration:
            self.get_logger().error(
                f'only {ratio:.0%} of depth pixels have been valid for {self.bad_duration:.0f} s, reopening the camera')
            self.failed = True


def main():
    rclpy.init()
    node = DepthWatchdog()
    try:
        while rclpy.ok() and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(1 if node.failed else 0)


if __name__ == '__main__':
    main()
