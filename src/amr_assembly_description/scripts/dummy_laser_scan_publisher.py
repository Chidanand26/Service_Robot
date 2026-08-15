#!/usr/bin/env python3
"""
Interactive Simulation LiDAR Node for AMR ServiceBot.
Raycasts against a virtual simulated environment (walls and obstacles)
using the robot's current TF pose so SLAM mapping works dynamically in simulation.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class DummyLaserScanPublisher(Node):
    def __init__(self):
        super().__init__('dummy_laser_scan_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan) # 10Hz

        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.pi / 180.0 # 360 beams
        self.range_min = 0.15
        self.range_max = 12.0

        # Define a virtual environment: line segments representing walls/obstacles [(x1,y1, x2,y2)]
        self.walls = [
            # Outer room boundaries (8m x 8m room)
            (-4.0, -4.0, 4.0, -4.0),
            (4.0, -4.0, 4.0, 4.0),
            (4.0, 4.0, -4.0, 4.0),
            (-4.0, 4.0, -4.0, -4.0),
            # Interior room partition / obstacle 1
            (-1.5, -2.0, -1.5, 0.5),
            # Interior pillar / obstacle 2
            (1.5, 1.0, 2.5, 1.0),
            (2.5, 1.0, 2.5, 2.0),
            (2.5, 2.0, 1.5, 2.0),
            (1.5, 2.0, 1.5, 1.0),
        ]

        self.get_logger().info('Interactive Simulation LiDAR started on /scan (10 Hz, 360 beams, 8m room).')

    def publish_scan(self):
        # Get robot position and orientation in odom frame
        rx, ry, ryaw = 0.0, 0.0, 0.0
        try:
            t = self.tf_buffer.lookup_transform('odom', 'laser_frame', rclpy.time.Time())
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            # Extract yaw from quaternion
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            ryaw = 2.0 * math.atan2(qz, qw)
        except Exception:
            pass

        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        ranges = []

        for i in range(num_readings):
            local_angle = self.angle_min + i * self.angle_increment
            global_angle = ryaw + local_angle

            dx = math.cos(global_angle)
            dy = math.sin(global_angle)

            # Raycast against all walls: find minimum distance
            min_dist = self.range_max
            for (x1, y1, x2, y2) in self.walls:
                d = self._ray_segment_intersection(rx, ry, dx, dy, x1, y1, x2, y2)
                if d is not None and d < min_dist:
                    min_dist = d

            ranges.append(max(self.range_min, min_dist))

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        msg.intensities = [100.0] * num_readings

        self.publisher_.publish(msg)

    def _ray_segment_intersection(self, rx, ry, dx, dy, x1, y1, x2, y2):
        """Calculates distance along ray (rx,ry)+t*(dx,dy) to segment (x1,y1)-(x2,y2)."""
        denom = dx * (y2 - y1) - dy * (x2 - x1)
        if abs(denom) < 1e-6:
            return None

        t1 = ((x1 - rx) * (y2 - y1) - (y1 - ry) * (x2 - x1)) / denom
        t2 = ((x1 - rx) * dy - (y1 - ry) * dx) / denom

        if t1 > 0.05 and 0.0 <= t2 <= 1.0:
            return t1
        return None


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
