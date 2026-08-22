#!/usr/bin/env python3
"""
Simulation Odometry & Joint State Publisher for AMR ServiceBot.
Enables full interactive simulation driving without physical hardware:
- Subscribes to /cmd_vel
- Integrates diff-drive kinematics: (x, y, theta)
- Broadcasts dynamic TF: odom -> base_footprint
- Publishes /joint_states for wheel visualization
- Publishes /odom (nav_msgs/msg/Odometry)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class SimRobotOdometry(Node):
    def __init__(self):
        super().__init__('sim_robot_odometry')

        self.declare_parameter('wheel_radius', 0.070)
        self.declare_parameter('wheel_separation', 0.470)

        self.R = self.get_parameter('wheel_radius').value
        self.W = self.get_parameter('wheel_separation').value

        # Robot state in world (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Current commanded velocities
        self.v = 0.0
        self.w = 0.0

        # Wheel positions in radians
        self.pos_l = 0.0
        self.pos_r = 0.0

        self.last_time = self.get_clock().now()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # 30Hz update loop
        self.timer = self.create_timer(0.033, self.update_loop)

        self.get_logger().info('Simulation Odometry & Dynamics engine active. Listening to /cmd_vel.')

    def cmd_vel_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt > 0.5:
            dt = 0.033

        # Update pose
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt

        # Update wheel angles
        v_l = self.v - (self.w * self.W / 2.0)
        v_r = self.v + (self.w * self.W / 2.0)
        self.pos_l += (v_l / self.R) * dt
        self.pos_r += (v_r / self.R) * dt

        # Normalize theta
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Broadcast TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0

        # Quaternion from yaw
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # Publish Joint States
        js = JointState()
        js.header.stamp = current_time.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [float(self.pos_l), float(self.pos_r)]
        js.velocity = [float(v_l / self.R), float(v_r / self.R)]
        js.effort = [0.0, 0.0]
        self.joint_pub.publish(js)

        # Publish Odometry msg
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(self.v)
        odom.twist.twist.angular.z = float(self.w)
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SimRobotOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
