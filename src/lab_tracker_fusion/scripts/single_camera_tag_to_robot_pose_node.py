#!/usr/bin/env python3
"""
single_camera_tag_to_robot_pose_node

DEPRECATED PROTOTYPE:
This node uses apriltag_ros tag TF as tracking input. Keep it for the
single-camera prototype only; do not use it as the architecture boundary for
the multi-camera system.

Single-camera AprilTag ground-truth pipeline.

Subscribes to AprilTagDetectionArray, looks up the TF transform
(camera → tag) published by apriltag_ros, and computes the robot's
ground-truth pose in the lab world frame:

    T_lab_base = T_lab_cam  ·  T_cam_tag  ·  T_tag_base

where:
  T_lab_cam   – camera extrinsics loaded from camera_extrinsics.yaml
  T_cam_tag   – real-time pose from TF (published by apriltag_ros)
  T_tag_base  – inverse of the tag pose on the robot (from robots.yaml)

Publishes:
  /lab_gt/<robot_id>/pose  (geometry_msgs/PoseWithCovarianceStamped)
  /lab_gt/<robot_id>/odom  (nav_msgs/Odometry)
  TF: <world_frame> → <robot_id>/gt_base_link  (optional, never map/odom)
"""

import os
import yaml
import numpy as np

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node

import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, LookupException, \
    ConnectivityException, ExtrapolationException

import tf_transformations

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from apriltag_msgs.msg import AprilTagDetectionArray

from ament_index_python.packages import get_package_share_directory


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def pose_to_matrix(translation, quat_xyzw) -> np.ndarray:
    """[x,y,z] + [qx,qy,qz,qw] → 4×4 homogeneous transform matrix."""
    mat = tf_transformations.quaternion_matrix(quat_xyzw)   # returns 4×4
    mat[:3, 3] = translation
    return mat


def tf_stamped_to_matrix(tf: TransformStamped) -> np.ndarray:
    t = tf.transform.translation
    r = tf.transform.rotation
    return pose_to_matrix([t.x, t.y, t.z], [r.x, r.y, r.z, r.w])


def matrix_to_translation_and_quat(mat: np.ndarray):
    """4×4 matrix → ([x,y,z], [qx,qy,qz,qw])."""
    translation = mat[:3, 3]
    quat = tf_transformations.quaternion_from_matrix(mat)   # [x,y,z,w]
    return translation, quat


def diagonal_covariance(pos_var: float, ori_var: float) -> list:
    """Build a 6×6 diagonal covariance (row-major, 36 elements)."""
    cov = [0.0] * 36
    for i in range(3):
        cov[i * 7] = pos_var
    for i in range(3, 6):
        cov[i * 7] = ori_var
    return cov


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class SingleCameraTagToRobotPoseNode(Node):

    def __init__(self):
        super().__init__('single_camera_tag_to_robot_pose_node')

        # ---- parameter declarations ----------------------------------------
        pkg_share = get_package_share_directory('lab_tracker_fusion')
        default_extrinsics = os.path.join(
            pkg_share, 'configs', 'cameras', 'camera_extrinsics.yaml')
        default_robots = os.path.join(
            pkg_share, 'configs', 'tag_boards', 'robots.yaml')

        self.declare_parameter('detections_topic', '/cam_1/detections')
        self.declare_parameter('camera_id',        'cam_1')
        self.declare_parameter('camera_frame',     'cam_1_optical_frame')
        self.declare_parameter('tag_id',           10)
        self.declare_parameter('tag_frame',        'tag10')
        self.declare_parameter('robot_id',         'roxi_1')
        self.declare_parameter('world_frame',      'lab_map')
        self.declare_parameter('publish_tf',       True)
        self.declare_parameter('position_variance',    0.01)
        self.declare_parameter('orientation_variance', 0.01)
        self.declare_parameter('extrinsics_file',  default_extrinsics)
        self.declare_parameter('robots_file',      default_robots)

        # ---- read parameters -----------------------------------------------
        detections_topic   = self.get_parameter('detections_topic').value
        self.camera_id     = self.get_parameter('camera_id').value
        self.camera_frame  = self.get_parameter('camera_frame').value
        self.tag_id        = self.get_parameter('tag_id').value
        self.tag_frame     = self.get_parameter('tag_frame').value
        self.robot_id      = self.get_parameter('robot_id').value
        self.world_frame   = self.get_parameter('world_frame').value
        self.do_publish_tf = self.get_parameter('publish_tf').value
        pos_var            = self.get_parameter('position_variance').value
        ori_var            = self.get_parameter('orientation_variance').value
        extrinsics_file    = self.get_parameter('extrinsics_file').value
        robots_file        = self.get_parameter('robots_file').value

        # ---- static transforms from config ---------------------------------
        self.T_lab_cam  = self._load_camera_extrinsics(extrinsics_file)
        T_base_tag      = self._load_robot_tag(robots_file)
        # Pre-invert once; we need tag → base (the reverse direction).
        self.T_tag_base = np.linalg.inv(T_base_tag)

        # ---- covariance (constant, diagonal) --------------------------------
        self.covariance = diagonal_covariance(pos_var, ori_var)

        # ---- TF infrastructure ----------------------------------------------
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.do_publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # ---- publishers -----------------------------------------------------
        pose_topic = f'/lab_gt/{self.robot_id}/pose'
        odom_topic = f'/lab_gt/{self.robot_id}/odom'
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        self.odom_pub = self.create_publisher(Odometry,                  odom_topic, 10)

        # ---- subscriber -----------------------------------------------------
        self.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self._on_detections,
            10,
        )

        self.get_logger().info(
            f'Tracking tag {self.tag_id} ({self.tag_frame}) on {detections_topic}')
        self.get_logger().info(
            f'Publishing to {pose_topic} | {odom_topic} | TF={self.do_publish_tf}')
        if not self._configs_have_real_data(extrinsics_file, robots_file):
            self.get_logger().warn(
                'Config files contain placeholder values. '
                'Pose output is unreliable until extrinsics and tag positions are calibrated.')

    # ------------------------------------------------------------------ I/O

    def _load_camera_extrinsics(self, filepath: str) -> np.ndarray:
        with open(filepath) as f:
            data = yaml.safe_load(f)
        cam_cfg = data.get(self.camera_id, next(iter(data.values())))
        t = cam_cfg['transform']['translation']
        q = cam_cfg['transform']['rotation_quat_xyzw']
        return pose_to_matrix(t, q)

    def _load_robot_tag(self, filepath: str) -> np.ndarray:
        with open(filepath) as f:
            data = yaml.safe_load(f)
        robot_data = data[self.robot_id]
        for tag in robot_data['tags']:
            if tag['id'] == self.tag_id:
                t = tag['transform']['translation']
                q = tag['transform']['rotation_quat_xyzw']
                return pose_to_matrix(t, q)
        raise ValueError(
            f'Tag id={self.tag_id} not found for robot {self.robot_id} in {filepath}')

    @staticmethod
    def _configs_have_real_data(ext_file: str, robots_file: str) -> bool:
        """Heuristic: flag as placeholder if translation is all-zero/trivial."""
        try:
            with open(ext_file) as f:
                ext = yaml.safe_load(f)
            cam = next(iter(ext.values()))
            t = cam['transform']['translation']
            if t == [0.0, 0.0, 0.0] or t == [0.0, 0.0, 2.0]:
                return False
        except Exception:
            pass
        return True

    # ---------------------------------------------------------- callback

    def _on_detections(self, msg: AprilTagDetectionArray):
        # Gate: only process if our tag is in this frame.
        if not any(d.id == self.tag_id for d in msg.detections):
            return

        stamp = msg.header.stamp
        try:
            tf_cam_tag = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.tag_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'TF lookup {self.camera_frame}→{self.tag_frame} failed: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        # T_lab_base = T_lab_cam · T_cam_tag · T_tag_base
        T_cam_tag  = tf_stamped_to_matrix(tf_cam_tag)
        T_lab_base = self.T_lab_cam @ T_cam_tag @ self.T_tag_base

        translation, quat = matrix_to_translation_and_quat(T_lab_base)

        self._publish_pose(translation, quat, stamp)
        self._publish_odom(translation, quat, stamp)
        if self.do_publish_tf:
            self._publish_tf(translation, quat, stamp)

    # ---------------------------------------------------------- publishers

    def _fill_pose(self, msg_pose, t, q):
        msg_pose.position.x    = float(t[0])
        msg_pose.position.y    = float(t[1])
        msg_pose.position.z    = float(t[2])
        msg_pose.orientation.x = float(q[0])
        msg_pose.orientation.y = float(q[1])
        msg_pose.orientation.z = float(q[2])
        msg_pose.orientation.w = float(q[3])

    def _publish_pose(self, t, q, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.world_frame
        self._fill_pose(msg.pose.pose, t, q)
        msg.pose.covariance = self.covariance
        self.pose_pub.publish(msg)

    def _publish_odom(self, t, q, stamp):
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.world_frame
        msg.child_frame_id  = f'{self.robot_id}/gt_base_link'
        self._fill_pose(msg.pose.pose, t, q)
        msg.pose.covariance = self.covariance
        self.odom_pub.publish(msg)

    def _publish_tf(self, t, q, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = stamp
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id  = f'{self.robot_id}/gt_base_link'
        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])
        tf_msg.transform.rotation.x    = float(q[0])
        tf_msg.transform.rotation.y    = float(q[1])
        tf_msg.transform.rotation.z    = float(q[2])
        tf_msg.transform.rotation.w    = float(q[3])
        self.tf_broadcaster.sendTransform(tf_msg)


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = SingleCameraTagToRobotPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
