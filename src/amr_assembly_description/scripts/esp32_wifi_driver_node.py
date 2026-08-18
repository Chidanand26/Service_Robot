#!/usr/bin/env python3
"""
ROS 2 Driver Node for ESP32 CS-D508 Controller over Wi-Fi (UDP).

Features:
- Receives UDP telemetry at 50Hz: "P <pos1_rev> <pos2_rev> <busy1> <busy2> <alm1> <alm2>"
- Publishes /joint_states (sensor_msgs/msg/JointState)
- Computes Differential Drive Odometry & broadcasts dynamic TF: odom -> base_footprint
- Publishes /odom (nav_msgs/msg/Odometry)
- Subscribes to /cmd_pos (std_msgs/msg/Float32MultiArray) and sends UDP commands "M <rev1> <rev2>\n"
"""

import math
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


class ESP32WiFiDriver(Node):
    def __init__(self):
        super().__init__('esp32_wifi_driver')

        # Declare parameters
        self.declare_parameter('udp_port', 8888)
        self.declare_parameter('esp32_ip', '10.78.37.133')
        self.declare_parameter('joint_name_1', 'left_wheel_joint')
        self.declare_parameter('joint_name_2', 'right_wheel_joint')
        self.declare_parameter('wheel_radius', 0.070)       # 70mm radius
        self.declare_parameter('wheel_separation', 0.370)   # 370mm baseline
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('suppress_alarm_errors', True)

        self.port = self.get_parameter('udp_port').value
        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.j1_name = self.get_parameter('joint_name_1').value
        self.j2_name = self.get_parameter('joint_name_2').value
        self.R = self.get_parameter('wheel_radius').value
        self.W = self.get_parameter('wheel_separation').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.suppress_alm = self.get_parameter('suppress_alarm_errors').value

        self._alm_warned_once = False

        # Odometry state (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_pos1_rev = None
        self.prev_pos2_rev = None
        self.last_time = self.get_clock().now()

        # Setup UDP socket listener
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.setblocking(False)

        # ROS 2 Publishers & Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_sub = self.create_subscription(
            Float32MultiArray, 'cmd_pos', self.cmd_pos_callback, 10
        )

        # 50Hz timer to process UDP packets
        self.timer = self.create_timer(0.02, self.read_udp_telemetry)
        # Keep-alive TF: re-broadcast odom->base_footprint every 100ms
        # so TF cache never expires when robot is stationary
        self.tf_keepalive_timer = self.create_timer(0.1, self._publish_odom_tf)

        # Publish identity TF immediately so SLAM/RViz can find laser_frame
        # before the first UDP packet arrives from ESP32
        self._publish_odom_tf()

        self.get_logger().info(
            f'ESP32 Wi-Fi (UDP) Driver active on port {self.port}. '
            f'Target ESP32 IP: {self.esp32_ip}. Real-time odometry enabled.'
        )

    def _publish_odom_tf(self):
        """Publish current odom->base_footprint TF. Called on init and after each encoder update."""
        if not self.publish_tf:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

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
        """Receive UDP telemetry packets from ESP32 and compute real-time odometry."""
        while True:
            try:
                data, addr = self.sock.recvfrom(256)
                self.esp32_ip = addr[0]
                line = data.decode('utf-8', errors='ignore').strip()

                if line.startswith('P '):
                    parts = line.split()
                    if len(parts) >= 7:
                        pos1_rev = float(parts[1])
                        pos2_rev = float(parts[2])
                        alm1 = int(parts[5])
                        alm2 = int(parts[6])

                        if (alm1 or alm2) and not self.suppress_alm:
                            self.get_logger().error(f'CS-D508 Alarm! Motor1: {alm1}, Motor2: {alm2}')

                        current_time = self.get_clock().now()
                        dt = (current_time - self.last_time).nanoseconds / 1e9
                        if dt <= 0.0:
                            dt = 0.02
                        self.last_time = current_time

                        # Differential Drive Odometry calculation
                        if self.prev_pos1_rev is not None and self.prev_pos2_rev is not None:
                            d_rev1 = pos1_rev - self.prev_pos1_rev
                            d_rev2 = pos2_rev - self.prev_pos2_rev

                            # Distance traveled by each wheel in meters
                            dL = d_rev1 * (2.0 * math.pi * self.R)
                            dR = d_rev2 * (2.0 * math.pi * self.R)

                            # Linear and angular displacement
                            d_dist = (dL + dR) / 2.0
                            d_theta = (dR - dL) / self.W

                            # Update robot pose (x, y, theta) using Runge-Kutta 2nd order (midpoint)
                            mid_theta = self.theta + (d_theta / 2.0)
                            self.x += d_dist * math.cos(mid_theta)
                            self.y += d_dist * math.sin(mid_theta)
                            self.theta += d_theta
                            self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

                            # Broadcast dynamic TF: odom -> base_footprint
                            self._publish_odom_tf()

                            # Publish Odometry message
                            odom = Odometry()
                            odom.header.stamp = current_time.to_msg()
                            odom.header.frame_id = 'odom'
                            odom.child_frame_id = 'base_footprint'
                            odom.pose.pose.position.x = float(self.x)
                            odom.pose.pose.position.y = float(self.y)
                            odom.pose.pose.position.z = 0.0
                            odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
                            odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
                            odom.twist.twist.linear.x = float(d_dist / dt)
                            odom.twist.twist.angular.z = float(d_theta / dt)
                            self.odom_pub.publish(odom)

                        self.prev_pos1_rev = pos1_rev
                        self.prev_pos2_rev = pos2_rev

                        # Publish JointState in radians
                        pos1_rad = pos1_rev * 2.0 * math.pi
                        pos2_rad = pos2_rev * 2.0 * math.pi

                        msg = JointState()
                        msg.header.stamp = current_time.to_msg()
                        msg.name = [self.j1_name, self.j2_name]
                        msg.position = [pos1_rad, pos2_rad]
                        msg.velocity = [0.0, 0.0]
                        msg.effort = [0.0, 0.0]
                        self.joint_pub.publish(msg)

            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().warn(f'Error processing UDP telemetry: {e}')
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
