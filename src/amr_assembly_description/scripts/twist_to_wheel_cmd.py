#!/usr/bin/env python3
"""
Twist to Wheel Command Node for AMR Assembly ServiceBot.
Converts /cmd_vel (geometry_msgs/msg/Twist) into target wheel revolution commands /cmd_pos (std_msgs/msg/Float32MultiArray)
for ESP32 motor controller.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class TwistToWheelCmd(Node):
    """
    Twist to Wheel Command Node for AMR Assembly ServiceBot.
    Converts /cmd_vel (Twist) into target wheel velocity commands /cmd_pos (Float32MultiArray [rps_left, rps_right])
    using a fixed 50Hz timer with true S-Curve (Jerk + Acceleration limited) motion profile.
    Guarantees zero initial jerk, spill-safe acceleration, and gentle deceleration.
    """
    def __init__(self):
        super().__init__('twist_to_wheel_cmd')

        # Robot kinematic parameters
        self.declare_parameter('wheel_radius', 0.070)       # 70mm radius
        self.declare_parameter('wheel_separation', 0.370)   # 370mm track width
        self.declare_parameter('control_rate_hz', 50.0)     # 50 Hz control loop

        self.R = self.get_parameter('wheel_radius').value
        self.W = self.get_parameter('wheel_separation').value
        self.rate_hz = self.get_parameter('control_rate_hz').value

        # Target command from teleop / navigation
        self.target_v = 0.0
        self.target_w = 0.0
        self.last_cmd_time = time.monotonic()

        # Smoothed state (Position, Velocity, Acceleration)
        self.v_smooth = 0.0
        self.a_v = 0.0

        self.w_smooth = 0.0
        self.a_w = 0.0

        # Motion limits — "Tea-Cup Safe" Ultra-Smooth Profile (Zero-jerk)
        # Linear motion limits
        self.max_accel_v = 0.25   # m/s² max linear acceleration
        self.max_jerk_v  = 0.75   # m/s³ max linear jerk (limits rate of change of acceleration)

        # Angular motion limits
        self.max_accel_w = 0.60   # rad/s² max angular acceleration
        self.max_jerk_w  = 1.50   # rad/s³ max angular jerk

        self.last_timer_time = time.monotonic()

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Float32MultiArray, 'cmd_pos', 10)
        self.twist_sub = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)

        # Fixed-rate control loop timer (50 Hz = 20ms)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._timer_callback)

        self.get_logger().info(
            f'Twist-to-Wheel converter active (Wheel R={self.R}m, W={self.W}m @ {self.rate_hz}Hz). '
            f'Ultra-smooth S-Curve motion profile active (Max a={self.max_accel_v}m/s², Jerk={self.max_jerk_v}m/s³).'
        )

    def twist_callback(self, msg: Twist):
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z
        self.last_cmd_time = time.monotonic()

    def _timer_callback(self):
        now = time.monotonic()
        dt = now - self.last_timer_time
        if dt <= 0.0 or dt > 0.1:
            dt = 1.0 / self.rate_hz
        self.last_timer_time = now

        # Watchdog: if no command received for > 0.25s, gently ramp target to 0
        if (now - self.last_cmd_time) > 0.25:
            self.target_v = 0.0
            self.target_w = 0.0

        # ─── S-Curve Linear Velocity Filter ────────────────────────────────────
        # Calculate desired acceleration toward target
        error_v = self.target_v - self.v_smooth
        desired_a_v = error_v / 0.12  # Time constant for exponential response
        desired_a_v = max(-self.max_accel_v, min(self.max_accel_v, desired_a_v))

        # Jerk-limit the acceleration change
        da_v = desired_a_v - self.a_v
        max_da_v = self.max_jerk_v * dt
        da_v = max(-max_da_v, min(max_da_v, da_v))
        self.a_v += da_v

        # Integrate velocity
        self.v_smooth += self.a_v * dt

        # Snap to 0 when near rest
        if abs(self.v_smooth) < 0.001 and abs(self.target_v) < 0.001:
            self.v_smooth = 0.0
            self.a_v = 0.0

        # ─── S-Curve Angular Velocity Filter ───────────────────────────────────
        error_w = self.target_w - self.w_smooth
        desired_a_w = error_w / 0.10
        desired_a_w = max(-self.max_accel_w, min(self.max_accel_w, desired_a_w))

        da_w = desired_a_w - self.a_w
        max_da_w = self.max_jerk_w * dt
        da_w = max(-max_da_w, min(max_da_w, da_w))
        self.a_w += da_w

        self.w_smooth += self.a_w * dt

        if abs(self.w_smooth) < 0.001 and abs(self.target_w) < 0.001:
            self.w_smooth = 0.0
            self.a_w = 0.0

        # ─── Differential Drive Kinematics ─────────────────────────────────────
        # v_left / v_right in m/s
        v_left = self.v_smooth - (self.w_smooth * self.W / 2.0)
        v_right = self.v_smooth + (self.w_smooth * self.W / 2.0)

        # Revolutions per second (RPS) = Velocity / Circumference
        rps_left = v_left / (2.0 * math.pi * self.R)
        rps_right = v_right / (2.0 * math.pi * self.R)

        cmd_msg = Float32MultiArray()
        cmd_msg.data = [float(rps_left), float(rps_right)]
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

