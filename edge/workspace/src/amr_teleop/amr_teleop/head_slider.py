"""実機の頭部(パン・チルト)をスライダーで動かす。"""
import time
import tkinter as tk

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from amr_teleop.arm_control import joint_limits


class HeadSlider(Node):
    def __init__(self):
        super().__init__('head_slider')
        self.joints = self.declare_parameter('joints', ['head_pan', 'head_tilt']).value
        self.move_time = self.declare_parameter('move_time', 0.3).value
        self.limits = None
        self.current = None
        self.pub = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_subscription(String, '/robot_description', self.on_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_description(self, msg):
        self.limits = joint_limits(msg.data, self.joints)

    def on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in self.joints):
            self.current = [pos[n] for n in self.joints]

    def send(self, positions):
        sec = int(self.move_time)
        point = JointTrajectoryPoint(positions=list(positions),
                                     time_from_start=Duration(sec=sec, nanosec=int((self.move_time - sec) * 1e9)))
        self.pub.publish(JointTrajectory(joint_names=self.joints, points=[point]))


def main():
    rclpy.init()
    node = HeadSlider()
    end = time.monotonic() + 10.0
    while (node.limits is None or node.current is None) and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.limits is None or node.current is None:
        node.get_logger().error('no /robot_description or head joint states; is the arm on the robot running?')
        rclpy.try_shutdown()
        return

    root = tk.Tk()
    root.title('head')
    shown = tk.StringVar()
    scales = []

    def update(_=None):
        values = [s.get() for s in scales]
        shown.set('  '.join(f'{n} {v:+.2f}' for n, v in zip(node.joints, values)))
        node.send(values)

    for name, (lo, hi), value in zip(node.joints, node.limits, node.current):
        vertical = name.endswith('tilt')
        scale = tk.Scale(root, label=name, from_=hi if vertical else lo, to=lo if vertical else hi, resolution=0.01,
                         length=360, orient=tk.VERTICAL if vertical else tk.HORIZONTAL)
        scale.set(round(value, 2))
        scale.pack(side=tk.LEFT, padx=8, pady=8)
        scales.append(scale)
    # 全部のスライダーがそろってから送る(途中で送ると関節の数が足りない指令になる)。
    for scale in scales:
        scale.configure(command=update)
    tk.Entry(root, textvariable=shown, width=36, state='readonly').pack(side=tk.BOTTOM, padx=8, pady=8)
    shown.set('  '.join(f'{n} {v:+.2f}' for n, v in zip(node.joints, node.current)))

    def spin():
        rclpy.spin_once(node, timeout_sec=0.0)
        root.after(20, spin)

    root.after(20, spin)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
