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
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_dir = get_package_share_directory('amr_assembly_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'amr_assembly_description.urdf')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'amr_assembly_description.rviz')
    slam_params_file = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')

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

    # 1b. Caster Joint State Publisher
    # The 4 caster wheels are fixed/passive - no motor drives them.
    # robot_state_publisher requires ALL non-fixed joints to be published
    # or it logs errors. We publish 0.0 for all passive joints here.
    caster_joint_state_publisher_node = Node(
        package='amr_assembly_description',
        executable='dummy_joint_state_publisher.py',
        name='caster_joint_state_publisher',
        output='screen',
    )

    # 2. Twist to Wheel Command Converter (/cmd_vel -> /cmd_pos)
    twist_to_wheel_cmd_node = Node(
        package='amr_assembly_description',
        executable='twist_to_wheel_cmd.py',
        name='twist_to_wheel_cmd',
        parameters=[{'wheel_radius': 0.080, 'wheel_separation': 0.427}],
        output='screen',
    )

    # 3. PS5 DualSense Controller (Bluetooth → /cmd_vel)
    #    joy_node reads /dev/input/js0 and publishes sensor_msgs/Joy
    #    teleop_twist_joy converts Joy → geometry_msgs/Twist on /cmd_vel
    #
    #    PS5 DualSense Button Mapping:
    #      Left Stick Y (axis 1)  → linear.x  (forward/backward)
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
            'scale_linear.x':  0.3,    # Normal speed: 0.3 m/s
            'scale_angular.yaw': 0.8,  # Normal turn:  0.8 rad/s
            'scale_linear_turbo.x': 0.6,   # Turbo speed: 0.6 m/s
            'scale_angular_turbo.yaw': 1.5, # Turbo turn:  1.5 rad/s
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
        parameters=[{'wheel_radius': 0.080, 'wheel_separation': 0.427}],
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
            'scan_mode': 'Sensitivity',
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
            'wheel_radius': 0.080,
            'wheel_separation': 0.427,
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
            'wheel_radius':     0.080,
            'wheel_separation': 0.427,
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


    # 5. SLAM Toolbox Lifecycle Node (delayed auto-configure + auto-activate)
    start_async_slam_toolbox_node = LifecycleNode(
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
        namespace=''
    )

    # Delayed CONFIGURE (wait 5s for all other nodes to start first)
    configure_event = TimerAction(
        period=5.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            ),
        ],
    )

    # Auto-ACTIVATE once CONFIGURE completes (inactive → active)
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[SLAM] Configuring done, auto-activating...'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
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
            'serial_port', default_value='/dev/rplidar',
            description='RPLiDAR serial port (permanent symlink via udev rules)',
        ),
        DeclareLaunchArgument(
            'esp_port', default_value='/dev/esp32',
            description='ESP32 serial port (permanent symlink via udev rules)',
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
            'enable_camera', default_value='true',
            description='Set to true to enable Intel RealSense D435i depth camera',
        ),

        GroupAction([
            PushRosNamespace(ns),
            robot_state_publisher_node,
            caster_joint_state_publisher_node,
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
            start_async_slam_toolbox_node,
            configure_event,
            activate_event,
            rviz_node,
        ]),
    ]

    return LaunchDescription(launch_actions)
