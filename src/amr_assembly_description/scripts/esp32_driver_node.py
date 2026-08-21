#!/usr/bin/env python3
"""
ROS 2 Serial Driver Node for ESP32 CS-D508 Motor Controller.

Communicates with ESP32 over USB Serial (e.g. /dev/ttyUSB0):
  - Telemetry from ESP32 at 50Hz: "P <pos1_rev> <pos2_rev> <busy1> <busy2> <alm1> <alm2>"
  - Commands sent to ESP32:       "M <rev1> <rev2>\\n"  (relative move)
                                  "S\\n"                 (stop)
                                  "Z\\n"                 (zero encoders)

ROS Outputs:
  - /joint_states  (left_wheel_joint, right_wheel_joint)
  - /odom          (nav_msgs/msg/Odometry)
  - TF:  odom -> base_footprint  (dynamic, 50Hz)

ROS Inputs:
  - /cmd_pos  (std_msgs/msg/Float32MultiArray: [left_revs, right_revs])
"""

import math
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


class ESP32SerialDriver(Node):
    def __init__(self):
        super().__init__('esp32_serial_driver')

        # Parameters
        self.declare_parameter('port',          '/dev/esp32')
        self.declare_parameter('baudrate',      115200)
        self.declare_parameter('joint_name_1',  'left_wheel_joint')
        self.declare_parameter('joint_name_2',  'right_wheel_joint')
        self.declare_parameter('wheel_radius',  0.070)    # metres (70mm)
        self.declare_parameter('wheel_separation', 0.370) # metres (370mm)
        self.declare_parameter('publish_tf',    True)
        self.declare_parameter('suppress_alarm_errors', True)

        self.port       = self.get_parameter('port').value
        self.baudrate   = self.get_parameter('baudrate').value
        self.j1_name    = self.get_parameter('joint_name_1').value
        self.j2_name    = self.get_parameter('joint_name_2').value
        self.R          = self.get_parameter('wheel_radius').value
        self.W          = self.get_parameter('wheel_separation').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.suppress_alm = self.get_parameter('suppress_alarm_errors').value

        # Odometry state
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.prev_pos1_rev = None
        self.prev_pos2_rev = None
        self.last_time = self.get_clock().now()

        # Serial port
        self.ser = None
        self._connect()

        # ROS interfaces
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub  = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub   = self.create_publisher(Odometry,   'odom',         10)
        self.imu_pub    = self.create_publisher(Imu,        'imu/data',    10)
        self.cmd_sub    = self.create_subscription(
            Float32MultiArray, 'cmd_pos', self._cmd_pos_callback, 10
        )

        # Timers
        self.read_timer = self.create_timer(0.02,  self._read_serial)   # 50 Hz read
        self.tf_timer   = self.create_timer(0.05,  self._publish_tf)    # 20 Hz TF keep-alive

        # Publish identity TF immediately (so SLAM can find laser_frame right away)
        self._publish_tf()

        self.get_logger().info(
            f'ESP32 Serial Driver started on {self.port} @ {self.baudrate} baud. '
            f'Real-time odometry + TF enabled.'
        )

    # ─── Serial Connection ──────────────────────────────────────────────────

    def _connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.02)
            self.get_logger().info(f'Connected to ESP32 on {self.port}')
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial connect failed: {e}. Will retry...')
            self.ser = None

    # ─── Command Sender ─────────────────────────────────────────────────────

    def _cmd_pos_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            # Send continuous velocity in RPS (revolutions/second)
            cmd = f'V {msg.data[0]:.4f} {msg.data[1]:.4f}\n'.encode()
            self._send(cmd)

    def _send(self, data: bytes):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')
                self.ser = None
        else:
            self._connect()

    # ─── Serial Reader + Odometry ───────────────────────────────────────────

    def _read_serial(self):
        if not self.ser or not self.ser.is_open:
            self._connect()
            return

        try:
            while self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                raw_bytes = self.ser.read_until(b'\n')
                if raw_bytes:
                    raw = raw_bytes.decode('utf-8', errors='ignore').strip()
                    if raw:
                        self._parse_telemetry(raw)
        except serial.SerialException:
            pass
        except Exception:
            pass

    def _parse_telemetry(self, line: str):
        """Parse "P pos1 pos2 busy1 busy2 alm1 alm2" and "I ax ay gz yaw pitch roll"."""
        if line.startswith('I '):
            parts = line.split()
            if len(parts) >= 7:
                try:
                    ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
                    yaw, pitch, roll = float(parts[4]), float(parts[5]), float(parts[6])

                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'imu_link'

                    # Convert RPY (radians) to Quaternion
                    cy = math.cos(yaw * 0.5)
                    sy = math.sin(yaw * 0.5)
                    cp = math.cos(pitch * 0.5)
                    sp = math.sin(pitch * 0.5)
                    cr = math.cos(roll * 0.5)
                    sr = math.sin(roll * 0.5)

                    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
                    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
                    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
                    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy

                    imu_msg.linear_acceleration.x = ax * 9.80665
                    imu_msg.linear_acceleration.y = ay * 9.80665
                    imu_msg.linear_acceleration.z = az * 9.80665

                    self.imu_pub.publish(imu_msg)
                except (ValueError, IndexError):
                    pass
            return

        if not line.startswith('P '):
            return
        parts = line.split()
        if len(parts) < 7:
            return

        try:
            pos1_rev = float(parts[1])
            pos2_rev = float(parts[2])
            alm1     = int(parts[5])
            alm2     = int(parts[6])
        except (ValueError, IndexError):
            return

        if (alm1 or alm2) and not self.suppress_alm:
            self.get_logger().error(f'CS-D508 Alarm! Motor1={alm1}, Motor2={alm2}')

        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 0.02
        self.last_time = now

        # Differential drive odometry
        if self.prev_pos1_rev is not None:
            dL = (pos1_rev - self.prev_pos1_rev) * (2.0 * math.pi * self.R)
            dR = (pos2_rev - self.prev_pos2_rev) * (2.0 * math.pi * self.R)

            d_dist  = (dL + dR) / 2.0
            d_theta = (dR - dL) / self.W

            mid_theta  = self.theta + d_theta / 2.0
            self.x    += d_dist * math.cos(mid_theta)
            self.y    += d_dist * math.sin(mid_theta)
            self.theta = (self.theta + d_theta + math.pi) % (2 * math.pi) - math.pi

            self._publish_tf()

            # /odom message
            odom = Odometry()
            odom.header.stamp    = now.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id  = 'base_footprint'
            odom.pose.pose.position.x  = float(self.x)
            odom.pose.pose.position.y  = float(self.y)
            odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
            odom.twist.twist.linear.x   = float(d_dist / dt)
            odom.twist.twist.angular.z  = float(d_theta / dt)
            self.odom_pub.publish(odom)

        self.prev_pos1_rev = pos1_rev
        self.prev_pos2_rev = pos2_rev

        # /joint_states
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name         = [self.j1_name, self.j2_name]
        js.position     = [pos1_rev * 2.0 * math.pi, pos2_rev * 2.0 * math.pi]
        js.velocity     = [0.0, 0.0]
        js.effort       = [0.0, 0.0]
        self.joint_pub.publish(js)

    # ─── TF Publisher ───────────────────────────────────────────────────────

    def _publish_tf(self):
        if not self.publish_tf:
            return
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation.z    = math.sin(self.theta / 2.0)
        t.transform.rotation.w    = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        if self.prev_pos1_rev is None:
            js = JointState()
            js.header.stamp = t.header.stamp
            js.name         = [self.j1_name, self.j2_name]
            js.position     = [0.0, 0.0]
            js.velocity     = [0.0, 0.0]
            js.effort       = [0.0, 0.0]
            self.joint_pub.publish(js)



def main(args=None):
    rclpy.init(args=args)
    node = ESP32SerialDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.write(b'S\n')   # Stop motors on shutdown
            node.ser.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
