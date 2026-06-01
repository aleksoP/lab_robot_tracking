"""
single_camera_tracking.launch.py

Milestone 1: single USB camera → AprilTag detector → ground-truth pose for roxi_1.

Pipeline:
  /dev/video0
    └─ v4l2_camera (namespace cam_1)
         publishes: /cam_1/image_raw, /cam_1/camera_info
         └─ apriltag_ros/apriltag_node (namespace cam_1)
              image_rect remapped → image_raw (no rectify until camera is calibrated)
              publishes: /cam_1/detections
                         TF: cam_1_optical_frame → tag10
              └─ single_camera_tag_to_robot_pose_node
                   publishes: /lab_gt/roxi_1/pose
                              /lab_gt/roxi_1/odom
                              TF: lab_map → roxi_1/gt_base_link

NOTE: image_proc/rectify_node stays disabled until real calibration exists.
Enable rectification at launch time with use_rectify:=true once cam_1_calibration.yaml
contains real intrinsics.

Camera pose estimates are unreliable until:
  1. Camera is calibrated and cam_1_calibration.yaml is populated.
  2. Camera extrinsics (camera_extrinsics.yaml) are measured.
  3. Tag position on robot (robots.yaml) is measured.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def make_camera_node(pkg_fusion, condition=None):
    return Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='cam_1',
        namespace='cam_1',
        parameters=[
            PathJoinSubstitution([pkg_fusion, 'configs', 'cameras', 'cam_1_v4l2.yaml']),
            {
                'video_device': LaunchConfiguration('device'),
                # v4l2_camera uses camera_info_url (not camera_calibration_file) to
                # override the default path derived from the camera's card name.
                'camera_info_url': [
                    'file://',
                    PathJoinSubstitution([
                        pkg_fusion, 'configs', 'cameras', 'cam_1_calibration.yaml'
                    ]),
                ],
            },
        ],
        output='screen',
        condition=condition,
    )


def generate_launch_description():
    pkg_bringup = FindPackageShare('lab_tracker_bringup')
    pkg_fusion   = FindPackageShare('lab_tracker_fusion')
    pkg_apriltag = FindPackageShare('apriltag_ros')

    # ---- arguments ---------------------------------------------------------
    device_arg = DeclareLaunchArgument(
        'device', default_value='/dev/video0',
        description='V4L2 device path for the USB camera')

    configure_camera_arg = DeclareLaunchArgument(
        'configure_camera', default_value='true',
        description='Apply manual V4L2 controls before starting the camera node')

    use_rectify_arg = DeclareLaunchArgument(
        'use_rectify', default_value='false',
        description='Rectify images before AprilTag detection; enable after real camera calibration exists')

    # ---- v4l2_camera -------------------------------------------------------
    # Uses a blocking capture thread (not a wall timer) so it reliably delivers
    # 30 fps from UVC cameras where usb_cam's timer-based select() drops frames.
    # Publishes /cam_1/image_raw and /cam_1/camera_info.
    configure_camera = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([pkg_bringup, 'scripts', 'set_camera_controls.sh']),
            LaunchConfiguration('device'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('configure_camera')),
    )
    usb_cam_node = make_camera_node(
        pkg_fusion, condition=UnlessCondition(LaunchConfiguration('configure_camera')))
    usb_cam_node_after_config = make_camera_node(
        pkg_fusion, condition=IfCondition(LaunchConfiguration('configure_camera')))
    start_camera_after_config = RegisterEventHandler(
        OnProcessExit(
            target_action=configure_camera,
            on_exit=[usb_cam_node_after_config],
        ),
        condition=IfCondition(LaunchConfiguration('configure_camera')),
    )

    # ---- image rectification (disabled until real calibration is available) --
    # With placeholder intrinsics (D=0) rectification is a no-op but burns CPU.
    # Enable this only after cam_1_calibration.yaml contains real intrinsics.
    use_rectify = LaunchConfiguration('use_rectify')
    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify',
        namespace='cam_1',
        remappings=[
            ('image',       'image_raw'),
            ('camera_info', 'camera_info'),
        ],
        output='screen',
        condition=IfCondition(use_rectify),
    )

    # ---- apriltag_ros detector ---------------------------------------------
    # When rectify is disabled: subscribe to image_raw directly (no distortion
    # correction, acceptable while D=0).
    # When rectify is enabled:  subscribe to image_rect (proper undistortion).
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='cam_1',
        remappings=[
            ('image_rect',  PythonExpression(["'image_rect' if '", use_rectify, "' == 'true' else 'image_raw'"])),
            ('camera_info', 'camera_info'),
            ('detections',  '/cam_1/detections'),
        ],
        parameters=[
            PathJoinSubstitution([pkg_fusion, 'configs', 'apriltag', 'tags.yaml']),
            # Match v4l2_camera's BEST_EFFORT sensor-data QoS on the image subscription.
            {'image_raw.qos_overrides./cam_1/image_raw.reliability': 'best_effort'},
        ],
        output='screen',
    )

    # ---- fusion node -------------------------------------------------------
    # Reads TF cam_1_optical_frame → tag10, applies extrinsics, publishes pose.
    fusion_node = Node(
        package='lab_tracker_fusion',
        executable='single_camera_tag_to_robot_pose_node',
        name='single_camera_tag_to_robot_pose_node',
        parameters=[
            # General fusion parameters (topics, IDs, covariance, …)
            PathJoinSubstitution([pkg_fusion, 'configs', 'fusion.yaml']),
            # Absolute paths to calibration config files (resolved at launch time)
            {
                'extrinsics_file': PathJoinSubstitution(
                    [pkg_fusion, 'configs', 'cameras', 'camera_extrinsics.yaml']),
                'robots_file': PathJoinSubstitution(
                    [pkg_fusion, 'configs', 'tag_boards', 'robots.yaml']),
            },
        ],
        output='screen',
    )

    nodes = [
        device_arg,
        configure_camera_arg,
        use_rectify_arg,
        configure_camera,
        start_camera_after_config,
        usb_cam_node,
        rectify_node,
        apriltag_node,
        fusion_node,
    ]
    return LaunchDescription(nodes)
