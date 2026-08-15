"""
Unified Mapping Launch File for servicebot.

Launches:
 1. Robot State Publisher (URDF & TFs)
 2. Joint State Publisher (Wi-Fi UDP Driver, Serial ESP32 Driver, or Fallback)
 3. Static Transform Publisher (odom -> base_link)
 4. RPLiDAR Driver Node or Synthetic Laser Scan (/scan)
 5. SLAM Toolbox (async mapping node -> /map)
 6. RViz2 (loaded with mapping.rviz)

Usage with Wi-Fi ESP32 & RPLiDAR:
    ros2 launch servicebot_description mapping.launch.py use_wifi:=true serial_port:=/dev/ttyUSB0 serial_baudrate:=115200

Usage without Hardware (Offline / Simulation mode):
    ros2 launch servicebot_description mapping.launch.py use_dummy_scan:=true
"""

import os
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('servicebot_description')

    xacro_file = os.path.join(pkg_dir, 'urdf', 'servicebot.urdf.xacro')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'mapping.rviz')

    ns = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    use_esp32 = LaunchConfiguration('use_esp32')
    use_wifi = LaunchConfiguration('use_wifi')
    use_dummy_scan = LaunchConfiguration('use_dummy_scan')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' prefix:=', prefix]),
        value_type=str
    )

    available_packages = get_packages_with_prefixes()

    nodes = [
        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # 2a. Static Transform: odom -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom_base_link',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'odom', '--child-frame-id', 'base_link'
            ],
            output='screen',
        ),

        # 2b. Static Transform: base_link -> lidar (explicit, matches URDF Rigid_3 joint)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_lidar',
            arguments=[
                '--x', '0.37', '--y', '0.2225', '--z', '0.254',
                '--roll', '0', '--pitch', '0', '--yaw', '-1.5708',
                '--frame-id', 'base_link', '--child-frame-id', 'lidar'
            ],
            output='screen',
        ),

        # 3a. Real Hardware RPLiDAR Node (when use_dummy_scan:=false)
        Node(
            condition=UnlessCondition(use_dummy_scan),
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': 'lidar',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity'
            }],
            output='screen',
        ),

        # 3b. Synthetic Laser Scan Publisher (when use_dummy_scan:=true)
        Node(
            condition=IfCondition(use_dummy_scan),
            package='servicebot_description',
            executable='dummy_laser_scan_publisher.py',
            name='dummy_laser_scan_publisher',
            output='screen',
        ),

        # 4a. ESP32 Wi-Fi UDP Driver (when use_wifi:=true)
        Node(
            condition=IfCondition(use_wifi),
            package='servicebot_description',
            executable='esp32_wifi_driver_node.py',
            name='esp32_wifi_driver',
            parameters=[{'udp_port': 8888}],
            output='screen',
        ),

        # 4b. ESP32 Serial Driver (when use_esp32:=true and use_wifi:=false)
        Node(
            condition=IfCondition(use_esp32),
            package='servicebot_description',
            executable='esp32_driver_node.py',
            name='esp32_driver',
            parameters=[{'port': '/dev/ttyUSB1', 'baudrate': 115200}],
            output='screen',
        ),

        # 4c. Dummy Joint State Publisher (when use_esp32:=false and use_wifi:=false)
        Node(
            condition=UnlessCondition(use_esp32),
            package='servicebot_description',
            executable='dummy_joint_state_publisher.py',
            name='dummy_joint_state_publisher',
            output='screen',
        ),
    ]

    # 5. SLAM Toolbox
    if 'slam_toolbox' in available_packages:
        nodes.append(
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[{
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'map_frame': 'map',
                    'scan_topic': '/scan',
                    'use_scan_matching': True,
                    'use_lifecycle_manager': False,
                    'use_multithread': True,
                    'mode': 'mapping',
                    'resolution': 0.05,
                    'max_laser_range': 12.0,
                    'minimum_time_interval': 0.1,
                    'transform_timeout': 0.5,
                    'tf_buffer_duration': 30.0,
                }],
                output='screen',
            )
        )

    # 6. RViz2
    if 'rviz2' in available_packages:
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_file],
                output='screen',
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='ROS 2 namespace',
        ),
        DeclareLaunchArgument(
            'prefix', default_value='',
            description='URDF prefix',
        ),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='RPLiDAR serial port',
        ),
        DeclareLaunchArgument(
            'serial_baudrate', default_value='115200',
            description='RPLiDAR baudrate (115200 for A1/A2, 256000 for A3/C1/S1)',
        ),
        DeclareLaunchArgument(
            'use_esp32', default_value='false',
            description='Set to true to use real ESP32 serial driver',
        ),
        DeclareLaunchArgument(
            'use_wifi', default_value='false',
            description='Set to true to use ESP32 Wi-Fi (UDP) driver',
        ),
        DeclareLaunchArgument(
            'use_dummy_scan', default_value='false',
            description='Set to true to publish simulated laser scans for testing without hardware',
        ),

        GroupAction([
            PushRosNamespace(ns),
            *nodes,
        ]),
    ])
