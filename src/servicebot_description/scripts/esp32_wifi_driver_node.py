#!/usr/bin/env python3
"""
ROS 2 Driver Node for ESP32 CS-D508 Controller over Wi-Fi (UDP).

- Receives UDP telemetry at 50Hz: "P <pos1_rev> <pos2_rev> <busy1> <busy2> <alm1> <alm2>"
- Publishes /joint_states (sensor_msgs/msg/JointState)
- Subscribes to /cmd_pos (std_msgs/msg/Float32MultiArray) and sends UDP commands "M <rev1> <rev2>\n"
"""

import math
import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class ESP32WiFiDriver(Node):
    def __init__(self):
        super().__init__('esp32_wifi_driver')

        # Declare parameters
        self.declare_parameter('udp_port', 8888)
        self.declare_parameter('esp32_ip', '10.78.37.133') # Adjust to your ESP32 IP address
        self.declare_parameter('joint_name_1', 'Revolute_1')
        self.declare_parameter('joint_name_2', 'Revolute_2')

        self.port = self.get_parameter('udp_port').value
        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.j1_name = self.get_parameter('joint_name_1').value
        self.j2_name = self.get_parameter('joint_name_2').value

        # Setup UDP socket listener
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.setblocking(False)

        # ROS 2 Publisher & Subscriber
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_sub = self.create_subscription(
            Float32MultiArray, 'cmd_pos', self.cmd_pos_callback, 10
        )

        # 50Hz timer to process UDP packets
        self.timer = self.create_timer(0.02, self.read_udp_telemetry)

        self.get_logger().info(f'ESP32 Wi-Fi (UDP) Driver started listening on UDP port {self.port}. Target ESP32 IP: {self.esp32_ip}')

    def cmd_pos_callback(self, msg: Float32MultiArray):
        """Send target move command to ESP32 over UDP: 'M <rev1> <rev2>\n'."""
        if len(msg.data) >= 2:
            r1 = msg.data[0]
            r2 = msg.data[1]
            cmd = f'M {r1:.4f} {r2:.4f}\n'.encode('utf-8')
            try:
                self.sock.sendto(cmd, (self.esp32_ip, self.port))
            except Exception as e:
                self.get_logger().error(f'Failed to send UDP command to ESP32: {e}')

    def read_udp_telemetry(self):
        """Receive UDP telemetry packets from ESP32."""
        while True:
            try:
                data, addr = self.sock.recvfrom(256)
                # Store ESP32 IP address dynamically when packet received
                self.esp32_ip = addr[0]
                line = data.decode('utf-8', errors='ignore').strip()

                if line.startswith('P '):
                    parts = line.split()
                    if len(parts) >= 7:
                        pos1_rev = float(parts[1])
                        pos2_rev = float(parts[2])
                        alm1 = int(parts[5])
                        alm2 = int(parts[6])

                        if alm1 or alm2:
                            self.get_logger().error(f'CS-D508 Alarm! Motor1: {alm1}, Motor2: {alm2}')

                        # Convert revolutions to radians
                        pos1_rad = pos1_rev * 2.0 * math.pi
                        pos2_rad = pos2_rev * 2.0 * math.pi

                        msg = JointState()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.name = [self.j1_name, self.j2_name]
                        msg.position = [pos1_rad, pos2_rad]
                        msg.velocity = [0.0, 0.0]
                        msg.effort = [0.0, 0.0]

                        self.joint_pub.publish(msg)
            except BlockingIOError:
                break # No more data available
            except Exception as e:
                self.get_logger().warn(f'Error reading UDP packet: {e}')
                break


def main(args=None):
    rclpy.init(args=args)
    node = ESP32WiFiDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
