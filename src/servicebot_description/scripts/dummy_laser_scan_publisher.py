#!/usr/bin/env python3
"""
Dummy Laser Scan Publisher for testing SLAM and RViz without physical LiDAR hardware.
Publishes synthetic LaserScan messages on /scan at 10Hz.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class DummyLaserScanPublisher(Node):
    def __init__(self):
        super().__init__('dummy_laser_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan) # 10Hz
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.pi / 180.0 # 360 samples
        self.get_logger().info('Dummy Laser Scan Publisher started on /scan (10 Hz).')

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar'
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.15
        msg.range_max = 12.0

        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        # Create a simulated square room around the robot (3m x 3m)
        ranges = []
        for i in range(num_readings):
            angle = self.angle_min + i * self.angle_increment
            # Distance to a 3m square box wall
            cos_a = abs(math.cos(angle))
            sin_a = abs(math.sin(angle))
            dist = 1.5 / max(cos_a, sin_a, 1e-4)
            ranges.append(min(dist, 12.0))

        msg.ranges = ranges
        msg.intensities = [100.0] * num_readings
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyLaserScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
