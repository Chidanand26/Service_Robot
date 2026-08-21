"""
Unified Mapping Launch File for amr_assembly_description.

Supports:
 1. Interactive Simulation Mode (Offline):
    ros2 launch amr_assembly_description mapping.launch.py use_dummy_scan:=true

 2. Real Hardware Mode (ESP32 Wi-Fi + RPLiDAR on /dev/ttyUSB0):
    ros2 launch amr_assembly_description mapping.launch.py use_wifi:=true serial_port:=/dev/ttyUSB0 serial_baudrate:=115200
"""

import os
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def resolve_ports():
    """Return default port values. Override at launch via serial_port:= and esp_port:= args."""
    if os.path.exists('/dev/rplidar') and os.path.exists('/dev/esp32'):
        return '/dev/rplidar', '/dev/esp32'
    return '/dev/rplidar', '/dev/esp32'



def generate_launch_description():
    pkg_dir = get_package_share_directory('amr_assembly_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'amr_assembly_description.urdf')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'amr_assembly_description.rviz')
    slam_params_file = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')

    auto_lidar, auto_esp = resolve_ports()

    with open(urdf_file, 'r') as f:
        robot_description_config = f.read()

    ns = LaunchConfiguration('namespace')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    use_esp32 = LaunchConfiguration('use_esp32')
    use_wifi = LaunchConfiguration('use_wifi')
    use_dummy_scan = LaunchConfiguration('use_dummy_scan')
    enable_camera = LaunchConfiguration('enable_camera')

    available_packages = get_packages_with_prefixes()

    # 1. Robot State Publisher (URDF & TF Tree)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config}],
        output='screen',
    )

    # 2. Twist to Wheel Command Converter (/cmd_vel -> /cmd_pos)
    twist_to_wheel_cmd_node = Node(
        package='amr_assembly_description',
        executable='twist_to_wheel_cmd.py',
        name='twist_to_wheel_cmd',
        parameters=[{'wheel_radius': 0.070, 'wheel_separation': 0.370}],
        output='screen',
    )

    # 3. PS5 DualSense Controller (Bluetooth → /cmd_vel)
    #    joy_node reads /dev/input/js0 and publishes sensor_msgs/Joy
    #    teleop_twist_joy converts Joy → geometry_msgs/Twist on /cmd_vel
    #
    #    PS5 DualSense Button Mapping:
    #      Left Stick Y (axis 1)  → linear.x  (inverted to match hardware direction)
    #      Left Stick X (axis 0)  → angular.z (turn left/right)
    #      L1 (button 4)          → enable button (HOLD to drive)
    #      R1 (button 5)          → turbo button (faster speed)
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
            'axis_linear.x':  1,       # Left stick Y axis
            'axis_angular.yaw': 0,     # Left stick X axis
            'scale_linear.x':  -0.25,  # Inverted: Push Up -> Forward (+0.25 m/s)
            'scale_angular.yaw': -0.60, # Inverted: Push Left -> Turn Left (-0.60 rad/s)
            'scale_linear_turbo.x': -0.45,   # Inverted Turbo: Push Up -> Forward (+0.45 m/s)
            'scale_angular_turbo.yaw': -1.0,  # Inverted Turbo: Push Left -> Turn Left (-1.00 rad/s)
            'enable_button': 4,        # L1 = enable (hold to drive)
            'enable_turbo_button': 5,  # R1 = turbo
            'require_enable_button': True,
        }],
        output='screen',
    )

    # --- SIMULATION MODE NODES (when use_dummy_scan:=true) ---
    sim_odometry_node = Node(
        condition=IfCondition(use_dummy_scan),
        package='amr_assembly_description',
        executable='sim_robot_odometry.py',
        name='sim_robot_odometry',
        parameters=[{'wheel_radius': 0.070, 'wheel_separation': 0.370}],
        output='screen',
    )

    sim_laser_node = Node(
        condition=IfCondition(use_dummy_scan),
        package='amr_assembly_description',
        executable='dummy_laser_scan_publisher.py',
        name='dummy_laser_scan_publisher',
        output='screen',
    )

    # --- REAL HARDWARE MODE NODES (when use_dummy_scan:=false) ---
    # Static TF fallback ONLY if esp32 is disabled (esp32_driver_node publishes dynamic odom->base_footprint when active)
    hardware_static_tf_node = Node(
        condition=UnlessCondition(use_esp32),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base_footprint',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom', '--child-frame-id', 'base_footprint'
        ],
        output='screen',
    )

    hardware_lidar_node = Node(
        condition=UnlessCondition(use_dummy_scan),
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

    hardware_esp32_wifi_node = Node(
        condition=IfCondition(use_wifi),
        package='amr_assembly_description',
        executable='esp32_wifi_driver_node.py',
        name='esp32_wifi_driver',
        parameters=[{
            'udp_port': 8888,
            'joint_name_1': 'left_wheel_joint',
            'joint_name_2': 'right_wheel_joint',
            'wheel_radius': 0.070,
            'wheel_separation': 0.370,
            'publish_tf': True,
            'suppress_alarm_errors': True
        }],
        output='screen',
    )

    hardware_esp32_serial_node = Node(
        condition=IfCondition(use_esp32),
        package='amr_assembly_description',
        executable='esp32_driver_node.py',
        name='esp32_serial_driver',
        parameters=[{
            'port':             LaunchConfiguration('esp_port'),
            'baudrate':         115200,
            'joint_name_1':     'left_wheel_joint',
            'joint_name_2':     'right_wheel_joint',
            'wheel_radius':     0.070,
            'wheel_separation': 0.370,
            'publish_tf':       True,
            'suppress_alarm_errors': True,
        }],
        output='screen',
    )

    # 4b. Intel RealSense D435i Camera Node
    realsense_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        condition=IfCondition(enable_camera),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': '',
            'initial_reset': 'false',
            'reconnect_timeout': '6.',
            'wait_for_device_timeout': '10.',
            'pointcloud.enable': 'true',
            'enable_depth': 'true',
            'enable_color': 'true',
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
            'base_frame_id': 'camera_link',
            'tf_prefix': '',
            'publish_tf': 'true',
        }.items(),
    )


    # 5. SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {
                'use_lifecycle_manager': False,
                'use_sim_time': False,
            }
        ],
        output='screen',
    )

    # 5b. SLAM Lifecycle Auto-Starter
    slam_lifecycle_starter_node = Node(
        package='amr_assembly_description',
        executable='slam_lifecycle_auto_starter.py',
        name='slam_lifecycle_auto_starter',
        output='screen',
    )

    # 6. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        output='screen',
    )

    launch_actions = [
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='ROS 2 namespace',
        ),
        DeclareLaunchArgument(
            'serial_port', default_value=auto_lidar,
            description='RPLiDAR serial port (auto-detected fallback)',
        ),
        DeclareLaunchArgument(
            'esp_port', default_value=auto_esp,
            description='ESP32 serial port (auto-detected fallback)',
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
            description='Set to true to run interactive simulation without hardware',
        ),
        DeclareLaunchArgument(
            'enable_camera', default_value='false',
            description='Set to true to enable Intel RealSense D435i depth camera (can be enabled if connected on USB 3.0)',
        ),

        GroupAction([
            PushRosNamespace(ns),
            robot_state_publisher_node,
            twist_to_wheel_cmd_node,
            joy_node,
            teleop_twist_joy_node,
            sim_odometry_node,
            sim_laser_node,
            hardware_static_tf_node,
            hardware_lidar_node,
            hardware_esp32_wifi_node,
            hardware_esp32_serial_node,
            realsense_camera_launch,
            slam_toolbox_node,
            slam_lifecycle_starter_node,
            rviz_node,
        ]),
    ]

    return LaunchDescription(launch_actions)
