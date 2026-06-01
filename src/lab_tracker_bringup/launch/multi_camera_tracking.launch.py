"""
multi_camera_tracking.launch.py

Scaffold launch for the future 6-camera lab ground-truth tracker.

This launch keeps camera-only calibration intact while establishing the correct
multi-camera system boundary:

  v4l2_camera -> apriltag_ros detections -> multi_camera_tracker_node

The central tracker consumes AprilTagDetectionArray messages directly and does
not use apriltag_ros tag TF frames as tracking input.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def enabled_condition(flag_name: str, index: int):
    return IfCondition(PythonExpression([
        "'",
        LaunchConfiguration(flag_name),
        "' == 'true' and ",
        LaunchConfiguration('camera_count'),
        " >= ",
        str(index),
    ]))


def make_camera_node(pkg_bringup, pkg_fusion, camera_id: str, device: str, index: int):
    return [
        ExecuteProcess(
            cmd=[
                PathJoinSubstitution([pkg_bringup, 'scripts', 'set_camera_controls.sh']),
                device,
            ],
            output='screen',
            condition=enabled_condition('use_cameras', index),
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name=camera_id,
            namespace=camera_id,
            parameters=[
                PathJoinSubstitution([pkg_fusion, 'configs', 'cameras', f'{camera_id}_v4l2.yaml']),
                {
                    'video_device': device,
                    'camera_info_url': [
                        'file://',
                        PathJoinSubstitution([
                            pkg_fusion, 'configs', 'cameras', f'{camera_id}_calibration.yaml'
                        ]),
                    ],
                },
            ],
            output='screen',
            condition=enabled_condition('use_cameras', index),
        ),
    ]


def make_apriltag_node(pkg_fusion, camera_id: str, index: int):
    return Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace=camera_id,
        remappings=[
            ('image_rect', 'image_raw'),
            ('camera_info', 'camera_info'),
            ('detections', f'/{camera_id}/detections'),
        ],
        parameters=[
            PathJoinSubstitution([pkg_fusion, 'configs', 'apriltag', 'tags_multi_camera.yaml']),
            {f'image_raw.qos_overrides./{camera_id}/image_raw.reliability': 'best_effort'},
        ],
        output='screen',
        condition=enabled_condition('use_apriltag', index),
    )


def generate_launch_description():
    pkg_bringup = FindPackageShare('lab_tracker_bringup')
    pkg_fusion = FindPackageShare('lab_tracker_fusion')

    use_cameras_arg = DeclareLaunchArgument(
        'use_cameras', default_value='true',
        description='Launch v4l2_camera nodes for the configured cameras')
    use_apriltag_arg = DeclareLaunchArgument(
        'use_apriltag', default_value='true',
        description='Launch apriltag_ros detector nodes for the configured cameras')
    camera_count_arg = DeclareLaunchArgument(
        'camera_count', default_value='6',
        description='Number of cameras to launch from cam_1 upward (maximum 6)')

    tracker_node = Node(
        package='lab_tracker_fusion',
        executable='multi_camera_tracker_node',
        name='multi_camera_tracker_node',
        parameters=[
            {
                'lab_layout_file': PathJoinSubstitution([pkg_fusion, 'configs', 'lab_layout.yaml']),
                'tracker_config_file': PathJoinSubstitution([pkg_fusion, 'configs', 'tracker.yaml']),
                'robots_file': PathJoinSubstitution([pkg_fusion, 'configs', 'robots', 'robots.yaml']),
                'cameras_dir': PathJoinSubstitution([pkg_fusion, 'configs', 'cameras']),
            },
        ],
        output='screen',
    )

    nodes = [
        use_cameras_arg,
        use_apriltag_arg,
        camera_count_arg,
    ]

    camera_devices = {
        'cam_1': '/dev/video0',
        'cam_2': '/dev/video2',
        'cam_3': '/dev/video4',
        'cam_4': '/dev/video6',
        'cam_5': '/dev/video8',
        'cam_6': '/dev/video10',
    }

    for index, (camera_id, device) in enumerate(camera_devices.items(), start=1):
        nodes.extend(make_camera_node(pkg_bringup, pkg_fusion, camera_id, device, index))
        nodes.append(make_apriltag_node(pkg_fusion, camera_id, index))

    nodes.append(tracker_node)
    return LaunchDescription(nodes)
