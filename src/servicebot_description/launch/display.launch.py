"""
Launch file for servicebot.

Usage:
    ros2 launch servicebot_description display.launch.py
    ros2 launch servicebot_description display.launch.py namespace:=robot1
"""

import os
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dir = get_package_share_directory('servicebot_description')

    xacro_file = os.path.join(pkg_dir, 'urdf', 'servicebot.urdf.xacro')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'display.rviz')
    controllers_file = os.path.join(pkg_dir, 'config', 'ros2_controllers.yaml')

    ns = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' prefix:=', prefix]),
        value_type=str
    )

    available_packages = get_packages_with_prefixes()

    nodes_to_launch = [
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
    ]

    if 'controller_manager' in available_packages:
        nodes_to_launch.extend([
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                name='controller_manager',
                parameters=[{'robot_description': robot_description}, controllers_file],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawn_joint_state_broadcaster',
                arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawn_position_controller',
                arguments=['position_controller', '--controller-manager', 'controller_manager'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawn_velocity_controller',
                arguments=['velocity_controller', '--controller-manager', 'controller_manager'],
                output='screen',
            ),
        ])
    elif 'joint_state_publisher_gui' in available_packages:
        nodes_to_launch.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
            )
        )
    elif 'joint_state_publisher' in available_packages:
        nodes_to_launch.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
            )
        )
    else:
        nodes_to_launch.append(
            Node(
                package='servicebot_description',
                executable='dummy_joint_state_publisher.py',
                name='joint_state_publisher',
                output='screen',
            )
        )

    if 'rviz2' in available_packages:
        nodes_to_launch.append(
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
            description='ROS 2 namespace for topics and nodes',
        ),
        DeclareLaunchArgument(
            'prefix', default_value='',
            description='URDF link/joint name prefix (passed to xacro)',
        ),

        GroupAction([
            PushRosNamespace(ns),
            *nodes_to_launch,
        ]),
    ])
