#!/usr/bin/env python3
"""
send_goal.py
------------
CLI tool to send a navigation goal to Nav2.

Usage:
    python3 send_goal.py --x 4.0 --y 2.0 --yaw 1.57
    python3 send_goal.py --x 1.5 --y 4.5 --yaw 0.0

Requires: ros2, nav2 running, map loaded.
"""

import sys
import argparse
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        self.get_logger().info(f'Waiting for NavigateToPose action server...')
        self._client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending goal: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}°')
        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb
        )
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        self.get_logger().info('Goal accepted. Robot is navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().info(f'  Distance remaining: {dist:.2f} m', throttle_duration_sec=2.0)

    def _result_cb(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info('✓ Goal reached successfully!')
        else:
            self.get_logger().warn(f'Goal ended with status: {status}')
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Send Nav2 goal')
    parser.add_argument('--x',   type=float, default=4.0, help='Goal X (metres)')
    parser.add_argument('--y',   type=float, default=1.5, help='Goal Y (metres)')
    parser.add_argument('--yaw', type=float, default=0.0, help='Goal yaw (radians)')
    args = parser.parse_args()

    rclpy.init()
    node = GoalSender()
    node.send_goal(args.x, args.y, args.yaw)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
