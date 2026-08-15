#!/usr/bin/env python3
"""
Caster / Passive Joint State Publisher for AMR ServiceBot.

robot_state_publisher requires ALL non-fixed joints (revolute/continuous/prismatic)
to have a value published on /joint_states, even if they are passive (no motor).

This node publishes zero-position states for:
  - caster_wheel1_joint (if present and non-fixed)
  - caster_wheel2_joint
  - caster_wheel3_joint
  - caster_wheel4_joint

The drive wheel joints (left_wheel_joint, right_wheel_joint) are handled
by the real ESP32 driver (hardware) or sim_robot_odometry (simulation).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CasterJointStatePublisher(Node):
    def __init__(self):
        super().__init__('caster_joint_state_publisher')

        # Publish zero-state for ALL non-fixed joints so robot_state_publisher
        # can always compute the full TF tree from the moment of launch.
        # The ESP32 driver (or sim_robot_odometry) overwrites the drive wheel
        # values with real data once connected — no conflict.
        self.passive_joints = [
            'left_wheel_joint',
            'right_wheel_joint',
            'caster_wheel1_joint',
            'caster_wheel2_joint',
            'caster_wheel3_joint',
            'caster_wheel4_joint',
        ]

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz is enough for static joints

        self.get_logger().info(
            f'Caster Joint State Publisher started. '
            f'Publishing zero-state for: {self.passive_joints}'
        )

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.passive_joints
        msg.position = [0.0] * len(self.passive_joints)
        msg.velocity = [0.0] * len(self.passive_joints)
        msg.effort   = [0.0] * len(self.passive_joints)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CasterJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
