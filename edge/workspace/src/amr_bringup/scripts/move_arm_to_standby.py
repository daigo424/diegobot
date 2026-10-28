#!/usr/bin/env python3
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class MoveArmToStandby(Node):
    def __init__(self):
        super().__init__('move_arm_to_standby')
        self.joints = self.declare_parameter('joints', ['']).value
        self.positions = self.declare_parameter('positions', [0.0]).value
        self.max_velocity = self.declare_parameter('max_velocity', 0.5).value
        self.min_duration = self.declare_parameter('min_duration', 2.0).value
        self.current = {}
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.client = ActionClient(
            self, FollowJointTrajectory, '/arm_group_controller/follow_joint_trajectory')

    def on_joint_states(self, msg):
        self.current.update(zip(msg.name, msg.position))

    def run(self):
        if not self.client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error('arm_group_controller is not available')
            return False
        # /joint_statesには車輪だけのメッセージも流れるため、腕の全関節が揃うまで待つ。
        while rclpy.ok() and not all(j in self.current for j in self.joints):
            rclpy.spin_once(self, timeout_sec=0.1)

        distance = max(abs(p - self.current[j]) for j, p in zip(self.joints, self.positions))
        duration = max(self.min_duration, distance / self.max_velocity)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=self.positions,
            time_from_start=Duration(sec=int(duration), nanosec=int(duration % 1 * 1e9)))]

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error('standby goal was rejected')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'standby failed: {result.error_string}')
            return False
        self.get_logger().info('arm is at standby')
        return True


def main():
    rclpy.init()
    node = MoveArmToStandby()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
