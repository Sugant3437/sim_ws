#!/usr/bin/env python3
"""
pose_relay.py
-------------
Subscribes to /robot_pose (PoseStamped in map frame) and re-broadcasts
the full TF chain required by Nav2:

    map → odom  (static identity)
    odom → base_footprint  (from camera pose)
    base_footprint → base_link  (static, z offset)

This node handles the localization interface that AMCL normally provides.
Nav2 expects:
  - map frame
  - odom → base_footprint transform at high frequency
  - /robot_pose or pose available via TF

No AMCL, No MCPL – pure camera-based localization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from nav_msgs.msg import Odometry


class PoseRelay(Node):

    WHEEL_RADIUS = 0.045   # must match URDF

    def __init__(self):
        super().__init__('pose_relay')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish Odometry so Nav2's controller has velocity feedback
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.create_subscription(PoseStamped, '/robot_pose',
                                 self._pose_cb, qos)

        # Static: map → odom (identity – camera provides global pose)
        self._pub_static_map_odom()
        # Static: base_footprint → base_link
        self._pub_static_footprint_base()

        self._prev_x = None
        self._prev_y = None
        self._prev_theta = None
        self._prev_stamp = None

        self.get_logger().info('Pose relay started – bridging /robot_pose → TF + /odom')

    def _pub_static_map_odom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def _pub_static_footprint_base(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.translation.z = self.WHEEL_RADIUS
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def _pose_cb(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        # Extract yaw from quaternion
        theta = 2.0 * math.atan2(q.z, q.w)

        stamp = msg.header.stamp

        # ── Broadcast odom → base_footprint ──
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        self.tf_broadcaster.sendTransform(t)

        # ── Publish /odom message for Nav2 ──
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = q
        # Compute velocity from finite differences
        if self._prev_stamp is not None:
            dt = (stamp.sec - self._prev_stamp.sec) + \
                 (stamp.nanosec - self._prev_stamp.nanosec) * 1e-9
            if dt > 1e-6:
                dx = x - self._prev_x
                dy = y - self._prev_y
                dtheta = theta - self._prev_theta
                odom.twist.twist.linear.x = math.sqrt(dx*dx + dy*dy) / dt
                odom.twist.twist.angular.z = dtheta / dt
        # Small pose covariance (camera is reliable)
        cov = [0.0] * 36
        cov[0] = 0.01; cov[7] = 0.01; cov[35] = 0.01
        odom.pose.covariance = cov
        odom.twist.covariance = cov
        self.odom_pub.publish(odom)

        self._prev_x = x
        self._prev_y = y
        self._prev_theta = theta
        self._prev_stamp = stamp


def main(args=None):
    rclpy.init(args=args)
    node = PoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
