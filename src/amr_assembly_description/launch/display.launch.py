import os

from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("amr_assembly_description")
    urdf = os.path.join(pkg, "urdf", "amr_assembly_description.urdf")
    with open(urdf) as f:
        robot_description = f.read()

    display_rviz = os.path.join(pkg, "rviz", "display.rviz")
    if not os.path.exists(display_rviz):
        display_rviz = os.path.join(pkg, "rviz", "amr_assembly_description.rviz")

    available_packages = get_packages_with_prefixes()
    if "joint_state_publisher_gui" in available_packages:
        jsp_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        )
    elif "joint_state_publisher" in available_packages:
        jsp_node = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
        )
    else:
        jsp_node = Node(
            package="amr_assembly_description",
            executable="dummy_joint_state_publisher.py",
            output="screen",
        )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        jsp_node,
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", display_rviz],
        ),
    ])
