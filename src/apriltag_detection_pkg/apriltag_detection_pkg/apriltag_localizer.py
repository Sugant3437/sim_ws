#!/usr/bin/env python3
"""
apriltag_localizer.py
---------------------
Subscribes to the overhead camera image, detects AprilTag ID 0 on the robot,
estimates (x, y, theta) in the map frame, and:
  - Publishes  /robot_pose  (geometry_msgs/PoseStamped)
  - Broadcasts TF:  map -> odom -> base_footprint
  - Publishes  /initialpose for Nav2 localization

Camera is mounted at (3.0, 3.0, 2.55) pointing straight down.
Tag is on top of the robot at base_link + ~0.16m height.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import cv2
import numpy as np
import math

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from builtin_interfaces.msg import Time


# ──────────────────────────────────────────────────────────────
#  AprilTag detector wrapper (uses cv2.aruco / pupil-apriltags)
# ──────────────────────────────────────────────────────────────
try:
    import apriltag as _apriltag_lib
    _USE_PUPIL = True
except ImportError:
    _USE_PUPIL = False

class TagDetector:
    """Thin wrapper around either pupil-apriltags or cv2.aruco."""

    def __init__(self, tag_family="tag36h11"):
        if _USE_PUPIL:
            self.detector = _apriltag_lib.Detector(
                families=tag_family,
                nthreads=2,
                quad_decimate=2.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0
            )
            self._use_pupil = True
        else:
            # Fallback: OpenCV ArUco (AprilTag dictionaries available in 4.x)
            self._aruco_dict = cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_APRILTAG_36h11)
            self._aruco_params = cv2.aruco.DetectorParameters_create()
            self._use_pupil = False

    def detect(self, gray_img):
        """
        Returns list of dicts:
          { 'id': int, 'corners': np.array (4,2) float32,
            'center': (cx, cy) }
        """
        detections = []
        if self._use_pupil:
            raw = self.detector.detect(gray_img)
            for d in raw:
                detections.append({
                    'id': d.tag_id,
                    'corners': np.array(d.corners, dtype=np.float32),
                    'center': d.center
                })
        else:
            corners_list, ids, _ = cv2.aruco.detectMarkers(
                gray_img, self._aruco_dict, parameters=self._aruco_params)
            if ids is not None:
                for corners, tag_id in zip(corners_list, ids.flatten()):
                    c = corners[0]  # shape (4,2)
                    cx = float(np.mean(c[:, 0]))
                    cy = float(np.mean(c[:, 1]))
                    detections.append({'id': int(tag_id), 'corners': c,
                                       'center': (cx, cy)})
        return detections


# ──────────────────────────────────────────────────────────────
#  Main ROS2 Node
# ──────────────────────────────────────────────────────────────
class AprilTagLocalizer(Node):

    # Camera position in map frame (metres)
    CAM_X = 3.0
    CAM_Y = 3.0
    CAM_Z = 2.55      # height above ground
    TAG_Z = 0.16      # height of tag above ground (base_link h + platform)

    # Real-world tag size (metres) — used for optional solvePnP
    TAG_SIZE = 0.10

    # Target tag ID
    TARGET_ID = 0

    def __init__(self):
        super().__init__('apriltag_localizer')

        # Parameters
        self.declare_parameter('camera_topic', '/overhead_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/overhead_camera/camera_info')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('tag_id', self.TARGET_ID)

        cam_topic = self.get_parameter('camera_topic').value
        cam_info_topic = self.get_parameter('camera_info_topic').value
        self.target_id = self.get_parameter('tag_id').value

        # Camera intrinsics (will be updated from CameraInfo)
        self.fx = 800.0
        self.fy = 800.0
        self.cx_img = 640.0
        self.cy_img = 360.0
        self.K = None

        self.bridge = CvBridge()
        self.detector = TagDetector("tag36h11")

        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Publishers
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', qos)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', qos)

        # Subscribers
        self.create_subscription(CameraInfo, cam_info_topic,
                                 self._camera_info_cb, 10)
        self.create_subscription(Image, cam_topic,
                                 self._image_cb, 10)

        # Publish static transform: map -> odom (identity; odometry not used)
        self._pub_static_map_odom()

        self._last_pose = None
        self._initialpose_sent = False

        self.get_logger().info(
            f'AprilTag Localizer started. '
            f'Listening on {cam_topic}, targeting tag ID {self.target_id}')

    # ── Static TF: map → odom (identity, odometry disabled) ──
    def _pub_static_map_odom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    # ── Camera info callback ──
    def _camera_info_cb(self, msg: CameraInfo):
        if self.K is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx_img = msg.k[2]
            self.cy_img = msg.k[5]
            self.K = np.array([[self.fx, 0, self.cx_img],
                                [0, self.fy, self.cy_img],
                                [0, 0, 1]], dtype=np.float64)
            self.get_logger().info(
                f'Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx_img:.1f} cy={self.cy_img:.1f}')

    # ── Image callback ──
    def _image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        stamp = msg.header.stamp

        for det in detections:
            if det['id'] != self.target_id:
                continue

            x_map, y_map, theta = self._pixel_to_map(det)
            self._publish_pose(x_map, y_map, theta, stamp)
            self._broadcast_tf(x_map, y_map, theta, stamp)

            # Send /initialpose once so Nav2 knows where to start
            if not self._initialpose_sent:
                self._send_initial_pose(x_map, y_map, theta, stamp)
                self._initialpose_sent = True
            break

    # ── Convert pixel detection → map frame pose ──
    def _pixel_to_map(self, det):
        cx, cy = det['center']

        # Camera is directly overhead, pointing down (-Z world = +Z optical)
        # effective_height = camera height - tag height
        eff_h = self.CAM_Z - self.TAG_Z

        # Back-project pixel to world
        # (cx - cx_img) / fx  gives normalised image coordinate
        dx = (cx - self.cx_img) / self.fx * eff_h
        dy = (cy - self.cy_img) / self.fy * eff_h

        # Camera frame: x=right, y=down in image → in world: x=right, y=forward
        # Camera is looking DOWN: image-x → world +x, image-y → world -y
        x_map = self.CAM_X + dx
        y_map = self.CAM_Y - dy   # image y is inverted vs world y

        # Estimate heading from tag corners
        corners = det['corners']   # (4,2): TL, TR, BR, BL  (AprilTag convention)
        # Use vector from corner 0 to corner 1 (top edge of tag in tag frame)
        tl = corners[0]; tr = corners[1]
        dx_tag = float(tr[0] - tl[0])
        dy_tag = float(tr[1] - tl[1])
        # In image coords y is down, in map coords y is up
        theta = math.atan2(-dy_tag, dx_tag)

        return x_map, y_map, theta

    # ── Build quaternion from yaw ──
    @staticmethod
    def _yaw_to_quat(theta):
        return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))

    # ── Publish /robot_pose ──
    def _publish_pose(self, x, y, theta, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        qx, qy, qz, qw = self._yaw_to_quat(theta)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)

    # ── Broadcast TF odom -> base_footprint ──
    def _broadcast_tf(self, x, y, theta, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = self._yaw_to_quat(theta)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    # ── Send initial pose to Nav2 ──
    def _send_initial_pose(self, x, y, theta, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = self._yaw_to_quat(theta)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Small covariance: camera provides reliable pose
        cov = [0.0] * 36
        cov[0] = 0.05   # x variance
        cov[7] = 0.05   # y variance
        cov[35] = 0.05  # yaw variance
        msg.pose.covariance = cov
        self.init_pub.publish(msg)
        self.get_logger().info(
            f'Sent /initialpose: x={x:.2f} y={y:.2f} θ={math.degrees(theta):.1f}°')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
