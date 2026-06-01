"""
camera_only.launch.py

Launches only the v4l2_camera node for a selected camera_id.
Use this for camera calibration or standalone streaming tests.

Publishes:
  /<camera_id>/image_raw   (sensor_msgs/Image)
  /<camera_id>/camera_info (sensor_msgs/CameraInfo)

Calibration workflow:
  ros2 launch lab_tracker_bringup camera_only.launch.py camera_id:=cam_1 device:=/dev/video0
  ros2 run camera_calibration cameracalibrator \\
    --size 8x6 --square 0.025 --no-service-check \\
    --ros-args --remap image:=/cam_1/image_raw --remap camera:=/cam_1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def make_camera_node(pkg_fusion, condition=None):
    camera_id = LaunchConfiguration('camera_id')
    return Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name=camera_id,
        namespace=camera_id,
        parameters=[
            PathJoinSubstitution([
                pkg_fusion,
                'configs',
                'cameras',
                PythonExpression(["'", camera_id, "_v4l2.yaml'"]),
            ]),
            {
                'video_device': LaunchConfiguration('device'),
                'camera_info_url': [
                    'file://',
                    PathJoinSubstitution([
                        pkg_fusion,
                        'configs',
                        'cameras',
                        PythonExpression(["'", camera_id, "_calibration.yaml'"]),
                    ]),
                ],
            },
        ],
        output='screen',
        condition=condition,
    )


def generate_launch_description():
    pkg_bringup = FindPackageShare('lab_tracker_bringup')
    pkg_fusion = FindPackageShare('lab_tracker_fusion')

    camera_id_arg = DeclareLaunchArgument(
        'camera_id', default_value='cam_1',
        description='Logical camera ID such as cam_1 through cam_6')

    device_arg = DeclareLaunchArgument(
        'device', default_value='/dev/video0',
        description='V4L2 device path for the USB camera')

    configure_camera_arg = DeclareLaunchArgument(
        'configure_camera', default_value='true',
        description='Apply manual V4L2 controls before starting the camera node')

    configure_camera = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([pkg_bringup, 'scripts', 'set_camera_controls.sh']),
            LaunchConfiguration('device'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('configure_camera')),
    )

    cam_node = make_camera_node(
        pkg_fusion, condition=UnlessCondition(LaunchConfiguration('configure_camera')))
    cam_node_after_config = make_camera_node(
        pkg_fusion, condition=IfCondition(LaunchConfiguration('configure_camera')))

    start_camera_after_config = RegisterEventHandler(
        OnProcessExit(
            target_action=configure_camera,
            on_exit=[cam_node_after_config],
        ),
        condition=IfCondition(LaunchConfiguration('configure_camera')),
    )

    return LaunchDescription([
        camera_id_arg,
        device_arg,
        configure_camera_arg,
        configure_camera,
        start_camera_after_config,
        cam_node,
    ])
