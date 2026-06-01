#!/usr/bin/env python3
"""
multi_camera_tracker_node

Central authoritative multi-camera ground-truth tracker.

Consumes AprilTagDetectionArray messages from each configured camera, estimates
camera_T_tag from tag corners plus per-camera intrinsics, converts those
observations into lab-frame robot base poses, and publishes only /lab_gt
outputs and optional gt TF frames.

This node intentionally does not use apriltag_ros tag TF frames as tracking
input, because global tag child frames collide in multi-camera deployments.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, Iterable

import cv2
import numpy as np
import yaml

import rclpy
import rclpy.time
from rclpy.node import Node

import tf2_ros
import tf_transformations

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory


def pose_to_matrix(translation, quat_xyzw) -> np.ndarray:
    mat = tf_transformations.quaternion_matrix(quat_xyzw)
    mat[:3, 3] = translation
    return mat


def translation_rpy_to_matrix(translation_xyz, rotation_rpy) -> np.ndarray:
    mat = tf_transformations.euler_matrix(*rotation_rpy)
    mat[:3, 3] = translation_xyz
    return mat


def matrix_to_translation_and_quat(mat: np.ndarray):
    translation = mat[:3, 3]
    quat = tf_transformations.quaternion_from_matrix(mat)
    return translation, quat


def diagonal_covariance_from_diag(diagonal: Iterable[float]) -> list[float]:
    values = list(diagonal)
    if len(values) != 6:
        raise ValueError('pose covariance diagonal must contain 6 elements')
    cov = [0.0] * 36
    for i, value in enumerate(values):
        cov[i * 7] = float(value)
    return cov


@dataclass
class CameraConfig:
    camera_id: str
    frame_id: str
    detections_topic: str
    image_topic: str
    camera_info_topic: str
    calibration_file: str
    extrinsic_parent_frame: str
    extrinsic_child_frame: str
    t_lab_camera: np.ndarray
    k: np.ndarray
    d: np.ndarray
    placeholder: bool


@dataclass
class RobotTagConfig:
    robot_id: str
    base_frame: str
    gt_frame: str
    tag_id: int
    size_m: float
    t_base_tag: np.ndarray
    t_tag_base: np.ndarray
    placeholder: bool


@dataclass
class Observation:
    camera_id: str
    camera_frame_id: str
    stamp_msg: object
    stamp_ns: int
    tag_id: int
    robot_id: str
    camera_t_tag: np.ndarray
    lab_t_base: np.ndarray
    score: float


class MultiCameraTrackerNode(Node):
    def __init__(self):
        super().__init__('multi_camera_tracker_node')

        pkg_share = get_package_share_directory('lab_tracker_fusion')
        default_lab_layout = os.path.join(pkg_share, 'configs', 'lab_layout.yaml')
        default_tracker = os.path.join(pkg_share, 'configs', 'tracker.yaml')
        default_robots = os.path.join(pkg_share, 'configs', 'robots', 'robots.yaml')
        default_cameras_dir = os.path.join(pkg_share, 'configs', 'cameras')

        self.declare_parameter('lab_layout_file', default_lab_layout)
        self.declare_parameter('tracker_config_file', default_tracker)
        self.declare_parameter('robots_file', default_robots)
        self.declare_parameter('cameras_dir', default_cameras_dir)

        lab_layout_file = self.get_parameter('lab_layout_file').value
        tracker_config_file = self.get_parameter('tracker_config_file').value
        robots_file = self.get_parameter('robots_file').value
        cameras_dir = self.get_parameter('cameras_dir').value

        self.package_share = pkg_share
        self.lab_layout = self._load_yaml(lab_layout_file)
        self.tracker_config = self._load_yaml(tracker_config_file).get('tracker', {})
        self.robot_tags = self._load_robot_tags(robots_file)
        self.lab_frame = self.lab_layout.get('lab_frame', 'lab_map')
        self.stale_timeout_sec = float(self.tracker_config.get('stale_timeout_sec', 0.25))
        self.publish_tf_enabled = bool(self.tracker_config.get('publish_tf', True))
        self.publish_odom_enabled = bool(self.tracker_config.get('publish_odom', True))
        self.pose_covariance = diagonal_covariance_from_diag(
            self.tracker_config.get(
                'pose_covariance_diagonal',
                [0.0004, 0.0004, 0.01, 0.01, 0.01, 0.0025],
            )
        )

        self.camera_configs = self._load_camera_configs(cameras_dir)
        self.observations: Dict[tuple[str, str], Observation] = {}
        self.pose_publishers: Dict[str, object] = {}
        self.odom_publishers: Dict[str, object] = {}
        self.robot_frames: Dict[str, str] = {}

        if self.publish_tf_enabled:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        for robot_tag in self.robot_tags.values():
            self.robot_frames[robot_tag.robot_id] = robot_tag.gt_frame

        for camera_id, config in self.camera_configs.items():
            self.create_subscription(
                AprilTagDetectionArray,
                config.detections_topic,
                lambda msg, camera_id=camera_id: self._on_detections(msg, camera_id),
                10,
            )

        self.publish_timer = self.create_timer(0.05, self._publish_best_observations)

        self.get_logger().info(
            f'Loaded {len(self.camera_configs)} camera configs from {cameras_dir}')
        self.get_logger().info(
            f'Loaded {len(self.robot_tags)} tag-to-robot mappings from {robots_file}')
        self.get_logger().info(
            f'Stale timeout: {self.stale_timeout_sec:.3f}s | publish_tf={self.publish_tf_enabled} | '
            f'publish_odom={self.publish_odom_enabled}')
        if any(config.placeholder for config in self.camera_configs.values()):
            self.get_logger().warn(
                'One or more camera configs contain placeholder extrinsics. '
                'Ground-truth output is not yet trustworthy.'
            )
        if any(tag.placeholder for tag in self.robot_tags.values()):
            self.get_logger().warn(
                'One or more robot tag transforms are placeholders. '
                'Ground-truth output is not yet trustworthy.'
            )

    def _resolve_path(self, path: str) -> str:
        if os.path.isabs(path):
            return path
        return os.path.join(self.package_share, path)

    def _load_yaml(self, path: str) -> dict:
        with open(path) as f:
            return yaml.safe_load(f) or {}

    def _load_camera_configs(self, cameras_dir: str) -> Dict[str, CameraConfig]:
        cameras_dir = self._resolve_path(cameras_dir)
        camera_ids = self.lab_layout.get('cameras', [])
        configs = {}
        for camera_id in camera_ids:
            path = os.path.join(cameras_dir, f'{camera_id}.yaml')
            data = self._load_yaml(path)
            if data.get('camera_id') != camera_id:
                self.get_logger().warn(
                    f'Camera config {path} declares camera_id={data.get("camera_id")!r}; '
                    f'expected {camera_id!r}'
                )
            calibration_file = self._resolve_path(data['calibration_file'])
            k, d = self._load_camera_intrinsics(calibration_file)
            extrinsic = data['extrinsic']
            t_lab_camera = translation_rpy_to_matrix(
                extrinsic['translation_xyz_m'],
                extrinsic['rotation_rpy_rad'],
            )
            configs[camera_id] = CameraConfig(
                camera_id=camera_id,
                frame_id=data['frame_id'],
                detections_topic=data['detections_topic'],
                image_topic=data['image_topic'],
                camera_info_topic=data['camera_info_topic'],
                calibration_file=calibration_file,
                extrinsic_parent_frame=extrinsic['parent_frame'],
                extrinsic_child_frame=extrinsic['child_frame'],
                t_lab_camera=t_lab_camera,
                k=k,
                d=d,
                placeholder=bool(extrinsic.get('placeholder', False)),
            )
        return configs

    def _load_camera_intrinsics(self, calibration_file: str) -> tuple[np.ndarray, np.ndarray]:
        data = self._load_yaml(calibration_file)
        camera_matrix = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape((3, 3))
        distortion = np.array(
            data.get('distortion_coefficients', {}).get('data', []),
            dtype=np.float64,
        )
        return camera_matrix, distortion

    def _load_robot_tags(self, robots_file: str) -> Dict[int, RobotTagConfig]:
        data = self._load_yaml(robots_file)
        robots = data.get('robots', {})
        tag_map = {}
        for robot_id, robot_data in robots.items():
            base_frame = robot_data['base_frame']
            gt_frame = robot_data['gt_frame']
            tags = robot_data.get('tags', {})
            for tag_id_raw, tag_data in tags.items():
                tag_id = int(tag_id_raw)
                base_t_tag_cfg = tag_data['base_T_tag']
                t_base_tag = translation_rpy_to_matrix(
                    base_t_tag_cfg['translation_xyz_m'],
                    base_t_tag_cfg['rotation_rpy_rad'],
                )
                tag_map[tag_id] = RobotTagConfig(
                    robot_id=robot_id,
                    base_frame=base_frame,
                    gt_frame=gt_frame,
                    tag_id=tag_id,
                    size_m=float(tag_data['size_m']),
                    t_base_tag=t_base_tag,
                    t_tag_base=np.linalg.inv(t_base_tag),
                    placeholder=bool(base_t_tag_cfg.get('placeholder', False)),
                )
        return tag_map

    def _on_detections(self, msg: AprilTagDetectionArray, camera_id: str) -> None:
        camera_config = self.camera_configs.get(camera_id)
        if camera_config is None:
            self.get_logger().warn(f'Ignoring detections from unknown camera {camera_id}')
            return

        stamp_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
        for detection in msg.detections:
            robot_tag = self.robot_tags.get(int(detection.id))
            if robot_tag is None:
                continue

            observation = self._make_observation(
                camera_config, robot_tag, detection, msg.header.stamp, stamp_ns)
            if observation is None:
                continue

            self.observations[(robot_tag.robot_id, camera_id)] = observation

    def _make_observation(
        self,
        camera_config: CameraConfig,
        robot_tag: RobotTagConfig,
        detection: AprilTagDetection,
        stamp_msg,
        stamp_ns: int,
    ) -> Observation | None:
        camera_t_tag = self._estimate_camera_t_tag(camera_config, robot_tag, detection)
        if camera_t_tag is None:
            return None

        lab_t_base = camera_config.t_lab_camera @ camera_t_tag @ robot_tag.t_tag_base
        score = float(detection.decision_margin)
        return Observation(
            camera_id=camera_config.camera_id,
            camera_frame_id=camera_config.frame_id,
            stamp_msg=stamp_msg,
            stamp_ns=stamp_ns,
            tag_id=int(detection.id),
            robot_id=robot_tag.robot_id,
            camera_t_tag=camera_t_tag,
            lab_t_base=lab_t_base,
            score=score,
        )

    def _estimate_camera_t_tag(
        self,
        camera_config: CameraConfig,
        robot_tag: RobotTagConfig,
        detection: AprilTagDetection,
    ) -> np.ndarray | None:
        half = robot_tag.size_m / 2.0
        object_points = np.array([
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ], dtype=np.float64)
        image_points = np.array(
            [[corner.x, corner.y] for corner in detection.corners],
            dtype=np.float64,
        )

        dist_coeffs = camera_config.d if camera_config.d.size > 0 else None
        flags = cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, 'SOLVEPNP_IPPE_SQUARE') else cv2.SOLVEPNP_ITERATIVE
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_config.k,
            dist_coeffs,
            flags=flags,
        )
        if not success:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                camera_config.k,
                dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        if not success:
            self.get_logger().warn(
                f'solvePnP failed for camera={camera_config.camera_id} tag={detection.id}',
                throttle_duration_sec=2.0,
            )
            return None

        rotation, _ = cv2.Rodrigues(rvec)
        camera_t_tag = np.eye(4, dtype=np.float64)
        camera_t_tag[:3, :3] = rotation
        camera_t_tag[:3, 3] = tvec.reshape(3)
        return camera_t_tag

    def _publish_best_observations(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        grouped: Dict[str, list[Observation]] = {}
        stale_keys = []

        for key, observation in self.observations.items():
            age_sec = (now_ns - observation.stamp_ns) / 1e9
            if age_sec > self.stale_timeout_sec:
                stale_keys.append(key)
                continue
            grouped.setdefault(observation.robot_id, []).append(observation)

        for key in stale_keys:
            self.observations.pop(key, None)

        for robot_id, observations in grouped.items():
            best = self._select_best_observation(observations)
            self._publish_observation(best)

    def _select_best_observation(self, observations: list[Observation]) -> Observation:
        # TODO: replace this with a better score that also considers tag pixel
        # area, reprojection error, camera center distance, and explicit
        # covariance estimation.
        return max(observations, key=lambda obs: (obs.score, obs.stamp_ns))

    def _ensure_pose_publisher(self, robot_id: str):
        if robot_id not in self.pose_publishers:
            topic = f'/lab_gt/{robot_id}/pose'
            self.pose_publishers[robot_id] = self.create_publisher(
                PoseWithCovarianceStamped, topic, 10)
        return self.pose_publishers[robot_id]

    def _ensure_odom_publisher(self, robot_id: str):
        if robot_id not in self.odom_publishers:
            topic = f'/lab_gt/{robot_id}/odom'
            self.odom_publishers[robot_id] = self.create_publisher(Odometry, topic, 10)
        return self.odom_publishers[robot_id]

    def _publish_observation(self, observation: Observation) -> None:
        translation, quat = matrix_to_translation_and_quat(observation.lab_t_base)

        pose_pub = self._ensure_pose_publisher(observation.robot_id)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = observation.stamp_msg
        pose_msg.header.frame_id = self.lab_frame
        pose_msg.pose.pose.position.x = float(translation[0])
        pose_msg.pose.pose.position.y = float(translation[1])
        pose_msg.pose.pose.position.z = float(translation[2])
        pose_msg.pose.pose.orientation.x = float(quat[0])
        pose_msg.pose.pose.orientation.y = float(quat[1])
        pose_msg.pose.pose.orientation.z = float(quat[2])
        pose_msg.pose.pose.orientation.w = float(quat[3])
        pose_msg.pose.covariance = self.pose_covariance
        pose_pub.publish(pose_msg)

        gt_frame = self.robot_frames[observation.robot_id]

        if self.publish_odom_enabled:
            odom_pub = self._ensure_odom_publisher(observation.robot_id)
            odom_msg = Odometry()
            odom_msg.header.stamp = observation.stamp_msg
            odom_msg.header.frame_id = self.lab_frame
            odom_msg.child_frame_id = gt_frame
            odom_msg.pose.pose = pose_msg.pose.pose
            odom_msg.pose.covariance = self.pose_covariance
            odom_pub.publish(odom_msg)

        if self.publish_tf_enabled:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = observation.stamp_msg
            tf_msg.header.frame_id = self.lab_frame
            tf_msg.child_frame_id = gt_frame
            tf_msg.transform.translation.x = float(translation[0])
            tf_msg.transform.translation.y = float(translation[1])
            tf_msg.transform.translation.z = float(translation[2])
            tf_msg.transform.rotation.x = float(quat[0])
            tf_msg.transform.rotation.y = float(quat[1])
            tf_msg.transform.rotation.z = float(quat[2])
            tf_msg.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
