#!/usr/bin/env python3
"""
Twist to Wheel Command Node for AMR Assembly ServiceBot.
Converts /cmd_vel (geometry_msgs/msg/Twist) into target wheel revolution commands /cmd_pos (std_msgs/msg/Float32MultiArray)
for ESP32 motor controller.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class TwistToWheelCmd(Node):
    def __init__(self):
        super().__init__('twist_to_wheel_cmd')

        # Parameters based on amr_assembly_description URDF
        self.declare_parameter('wheel_radius', 0.080)       # 80mm radius
        self.declare_parameter('wheel_separation', 0.427)   # Distance between left & right wheels
        self.declare_parameter('step_duration', 0.15)       # Time window per command (sec)

        self.R = self.get_parameter('wheel_radius').value
        self.W = self.get_parameter('wheel_separation').value
        self.dt = self.get_parameter('step_duration').value

        # Ultra-smooth "Tea-Cup Safe" S-Curve acceleration state
        self.v_smooth = 0.0
        self.w_smooth = 0.0

        # Smooth acceleration limits (m/s² and rad/s²)
        self.max_accel_v = 0.30   # Max linear acceleration: 0.30 m/s² (responsive & smooth)
        self.max_accel_w = 0.80   # Max angular acceleration: 0.80 rad/s²

        self.cmd_pub = self.create_publisher(Float32MultiArray, 'cmd_pos', 10)
        self.twist_sub = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)

        self.get_logger().info(
            f'Twist-to-Wheel converter started (Wheel R={self.R}m, W={self.W}m). '
            f'Ultra-smooth Tea-Cup safe motion profile active.'
        )

    def twist_callback(self, msg: Twist):
        target_v = msg.linear.x        # Target linear velocity (m/s)
        target_w = msg.angular.z       # Target angular velocity (rad/s)

        # Acceleration-limited velocity ramping (prevents current & momentum spikes)
        max_dv = self.max_accel_v * self.dt
        max_dw = self.max_accel_w * self.dt

        dv = target_v - self.v_smooth
        dw = target_w - self.w_smooth

        # Clamp acceleration change per step
        dv = max(-max_dv, min(max_dv, dv))
        dw = max(-max_dw, min(max_dw, dw))

        self.v_smooth += dv
        self.w_smooth += dw

        # Snap to 0 when near rest for clean stop
        if abs(self.v_smooth) < 0.002:
            self.v_smooth = 0.0
        if abs(self.w_smooth) < 0.002:
            self.w_smooth = 0.0

        # Differential Drive Kinematics with smoothed velocities
        v_left = self.v_smooth - (self.w_smooth * self.W / 2.0)
        v_right = self.v_smooth + (self.w_smooth * self.W / 2.0)

        # Distance moved in dt
        dist_left = v_left * self.dt
        dist_right = v_right * self.dt

        # Revolutions = Distance / (2 * pi * R)
        rev_left = dist_left / (2.0 * math.pi * self.R)
        rev_right = dist_right / (2.0 * math.pi * self.R)

        cmd_msg = Float32MultiArray()
        cmd_msg.data = [float(rev_left), float(rev_right)]
        self.cmd_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToWheelCmd()
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
