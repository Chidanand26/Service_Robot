#!/usr/bin/env python3
"""
Master Autonomous Navigation (Nav2) Launch File for AMR ServiceBot.
===================================================================
Hardware & Stack:
  - Robot Model: amr_assembly_description URDF (REP-103 standard)
  - Motor Driver: CS-D508 Steppers via ESP32 (/dev/esp32) + 50Hz S-Curve Controller
  - 2D LiDAR: RPLiDAR A1 (/dev/rplidar)
  - 3D Camera: Intel RealSense D435i
  - Localization: AMCL on saved map
  - Path Planning & Control: Nav2 Regulated Pure Pursuit Controller
  - Teleoperation: PS5 DualSense (Manual override ready)
"""

import os
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def resolve_ports():
    """Return verified port values using udev symlinks or hardware probe."""
    lidar_port = '/dev/rplidar' if os.path.exists('/dev/rplidar') else None
    esp_port = '/dev/esp32' if os.path.exists('/dev/esp32') else None

    if not lidar_port or not esp_port:
        import glob
        ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
        if len(ports) >= 2:
            if not lidar_port:
                lidar_port = ports[1]
            if not esp_port:
                esp_port = ports[0]
        elif len(ports) == 1:
            if not lidar_port:
                lidar_port = ports[0]
            if not esp_port:
                esp_port = ports[0]

    return lidar_port or '/dev/rplidar', esp_port or '/dev/esp32'



def generate_launch_description():
    pkg_dir = get_package_share_directory('amr_assembly_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'amr_assembly_description.urdf')
    default_map_file = os.path.join(pkg_dir, 'maps', 'service_room.yaml')
    default_nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'amr_assembly_description.rviz')

    auto_lidar, auto_esp = resolve_ports()

    with open(urdf_file, 'r') as f:
        robot_description_config = f.read()

    ns = LaunchConfiguration('namespace')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    esp_port = LaunchConfiguration('esp_port')
    use_esp32 = LaunchConfiguration('use_esp32')
    enable_camera = LaunchConfiguration('enable_camera')
    autostart = LaunchConfiguration('autostart')

    available_packages = get_packages_with_prefixes()

    # 1. Robot State Publisher (URDF TF broadcaster)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': False,
        }],
        output='screen',
    )

    # 2. 50Hz S-Curve Jerk-Limited Motion Controller
    twist_to_wheel_cmd_node = Node(
        package='amr_assembly_description',
        executable='twist_to_wheel_cmd.py',
        name='twist_to_wheel_cmd',
        parameters=[{
            'wheel_radius': 0.070,
            'wheel_separation': 0.470,
            'control_rate_hz': 50.0,
        }],
        output='screen',
    )

    # 3. Teleoperation (Joy + Teleop Joy for manual override)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        output='screen',
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[{
            'axis_linear.x': 1,
            'axis_angular.yaw': 0,
            'scale_linear.x': -0.25,  # Inverted: Push Up -> Forward (+0.25 m/s)
            'scale_angular.yaw': -0.60, # Inverted: Push Left -> Turn Left (-0.60 rad/s)
            'scale_linear_turbo.x': -0.45,  # Inverted Turbo: Push Up -> Forward (+0.45 m/s)
            'scale_angular_turbo.yaw': -1.0, # Inverted Turbo: Push Left -> Turn Left (-1.00 rad/s)
            'enable_button': 4,
            'enable_turbo_button': 5,
            'require_enable_button': True,
        }],
        output='screen',
    )

    # 4. Hardware Sensors & Drivers (Staggered to prevent serial port contention)
    hardware_esp32_serial_node = Node(
        condition=IfCondition(use_esp32),
        package='amr_assembly_description',
        executable='esp32_driver_node.py',
        name='esp32_driver_node',
        parameters=[{
            'port': esp_port,
            'baudrate': 115200,
            'wheel_radius': 0.070,
            'wheel_separation': 0.470,
            'publish_tf': True,
            'suppress_alarm_errors': True,
        }],
        output='screen',
    )

    hardware_lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'auto_reconnect': True,
            'scan_mode': 'Standard',
        }],
        output='screen',
    )

    # Delay LiDAR startup by 1.5s to let ESP32 settle and avoid USB bus contention
    delayed_lidar_launch = TimerAction(
        period=1.5,
        actions=[hardware_lidar_node]
    )

    # 5. Intel RealSense D435i Camera
    realsense_camera_launch = GroupAction(
        condition=IfCondition(enable_camera),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('realsense2_camera'),
                        'launch',
                        'rs_launch.py'
                    ])
                ),
                launch_arguments={
                    'camera_name': 'camera',
                    'tf_prefix': '',
                    'depth_module.profile': '640x480x15',
                    'rgb_camera.profile': '640x480x15',
                    'enable_depth': 'true',
                    'enable_color': 'true',
                    'pointcloud.enable': 'true',
                    'initial_reset': 'false',
                }.items(),
            )
        ] if 'realsense2_camera' in available_packages else []
    )

    # 6. Nav2 Bringup (Delayed by 3.0s until odom and TF are actively broadcasting)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
    )

    delayed_nav2_launch = TimerAction(
        period=3.0,
        actions=[nav2_bringup_launch]
    )

    # 7. RViz2 (Delayed by 3.5s)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        output='screen',
    )

    delayed_rviz_launch = TimerAction(
        period=3.5,
        actions=[rviz_node]
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('map', default_value=default_map_file, description='Full path to map yaml file'),
        DeclareLaunchArgument('params_file', default_value=default_nav2_params, description='Full path to Nav2 params file'),
        DeclareLaunchArgument('serial_port', default_value=auto_lidar, description='RPLiDAR port (auto-detected)'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200', description='RPLiDAR baudrate'),
        DeclareLaunchArgument('esp_port', default_value=auto_esp, description='ESP32 port (auto-detected)'),
        DeclareLaunchArgument('use_esp32', default_value='true', description='Enable ESP32 serial driver'),
        DeclareLaunchArgument('enable_camera', default_value='false', description='Enable RealSense D435i camera'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup Nav2 stack'),

        # Step 1: Core robot state & odometry TF (immediate)
        robot_state_publisher_node,
        twist_to_wheel_cmd_node,
        joy_node,
        teleop_twist_joy_node,
        hardware_esp32_serial_node,
        realsense_camera_launch,

        # Step 2: Delayed LiDAR startup (avoids USB initialization collision)
        delayed_lidar_launch,

        # Step 3: Delayed Nav2 stack (waits for stable odom & scan)
        delayed_nav2_launch,

        # Step 4: Delayed RViz2
        delayed_rviz_launch,
    ])
