#!/usr/bin/env python3
"""
ROS 2 Driver Node for ESP32 CS-D508 Serial Motor Controller.

Communicates with ESP32 over serial:
  - Telemetry received at 50Hz: "P <pos1_rev> <pos2_rev> <active1> <active2> <alm1> <alm2>"
  - Commands sent: "M <rev1> <rev2>\n", "S\n", "Z\n"
  - Publishes: /joint_states (sensor_msgs/msg/JointState)
  - Subscribes: /cmd_pos (std_msgs/msg/Float32MultiArray) or /joint_commands
"""

import math
import time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class ESP32CSD508Driver(Node):
    def __init__(self):
        super().__init__('esp32_csd508_driver')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('joint_name_1', 'Revolute_1')
        self.declare_parameter('joint_name_2', 'Revolute_2')
        self.declare_parameter('publish_rate_hz', 50.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.j1_name = self.get_parameter('joint_name_1').value
        self.j2_name = self.get_parameter('joint_name_2').value

        # Publishers & Subscribers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_sub = self.create_subscription(
            Float32MultiArray, 'cmd_pos', self.cmd_pos_callback, 10
        )

        self.ser = None
        self.connect_serial()

        # Timer for reading serial telemetry
        timer_period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(timer_period, self.read_serial_telemetry)

        self.get_logger().info(f'ESP32 CS-D508 Driver node started on {self.port} at {self.baudrate} baud.')

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            self.get_logger().info(f'Connected to ESP32 on {self.port}')
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial connection failed ({e}). Will retry automatically...')
            self.ser = None

    def cmd_pos_callback(self, msg: Float32MultiArray):
        """Send target revolution moves to ESP32: 'M <rev1> <rev2>'."""
        if len(msg.data) >= 2:
            r1 = msg.data[0]
            r2 = msg.data[1]
            cmd = f'M {r1:.4f} {r2:.4f}\n'
            self.send_serial(cmd)

    def send_serial(self, cmd_str: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd_str.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')
                self.ser = None
        else:
            self.connect_serial()

    def read_serial_telemetry(self):
        if not self.ser or not self.ser.is_open:
            self.connect_serial()
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('P '):
                    parts = line.split()
                    if len(parts) >= 7:
                        # P pos1 pos2 active1 active2 alm1 alm2
                        pos1_rev = float(parts[1])
                        pos2_rev = float(parts[2])
                        active1 = int(parts[3])
                        active2 = int(parts[4])
                        alm1 = int(parts[5])
                        alm2 = int(parts[6])

                        if alm1 or alm2:
                            self.get_logger().error(
                                f'CS-D508 Drive Alarm Detected! Motor1 Alarm: {alm1}, Motor2 Alarm: {alm2}'
                            )

                        # Convert revolutions to radians for ROS 2 JointState
                        pos1_rad = pos1_rev * 2.0 * math.pi
                        pos2_rad = pos2_rev * 2.0 * math.pi

                        msg = JointState()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.name = [self.j1_name, self.j2_name]
                        msg.position = [pos1_rad, pos2_rad]
                        msg.velocity = [0.0, 0.0]
                        msg.effort = [0.0, 0.0]

                        self.joint_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Error reading serial telemetry: {e}')
            self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = ESP32CSD508Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
