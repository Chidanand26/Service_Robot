#!/usr/bin/env python3
"""
SLAM Toolbox Lifecycle Auto-Starter
Guarantees slam_toolbox transitions unconfigured -> inactive -> active on ROS 2 Jazzy.
"""

import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State


class SlamLifecycleAutoStarter(Node):
    def __init__(self):
        super().__init__('slam_lifecycle_auto_starter')
        self.get_logger().info('SLAM Auto-Starter waiting for /slam_toolbox lifecycle services...')
        
        self.change_state_client = self.create_client(ChangeState, '/slam_toolbox/change_state')
        self.get_state_client = self.create_client(GetState, '/slam_toolbox/get_state')
        
        self.timer = self.create_timer(0.5, self._manage_lifecycle)
        self.step = 0
        self.retries = 0

    def _manage_lifecycle(self):
        if not self.change_state_client.wait_for_service(timeout_sec=0.2):
            self.retries += 1
            if self.retries % 6 == 0:
                self.get_logger().info('Waiting for /slam_toolbox service to become available...')
            return

        if self.step == 0:
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_CONFIGURE
            future = self.change_state_client.call_async(req)
            future.add_done_callback(self._configure_done)
            self.step = 1
            self.get_logger().info('Sent TRANSITION_CONFIGURE to /slam_toolbox')

    def _configure_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info('SLAM Toolbox configured successfully (now INACTIVE). Activating...')
                time.sleep(0.3)
                req = ChangeState.Request()
                req.transition.id = Transition.TRANSITION_ACTIVATE
                future_act = self.change_state_client.call_async(req)
                future_act.add_done_callback(self._activate_done)
            else:
                self.get_logger().warn('Failed to configure SLAM Toolbox. Retrying...')
                self.step = 0
        except Exception as e:
            self.get_logger().error(f'Configure error: {e}')
            self.step = 0

    def _activate_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info('✅ SLAM Toolbox is now ACTIVE! /map is publishing and TF map->odom is live.')
                self.timer.cancel()
            else:
                self.get_logger().warn('Failed to activate SLAM Toolbox. Retrying...')
                self.step = 0
        except Exception as e:
            self.get_logger().error(f'Activate error: {e}')
            self.step = 0


def main(args=None):
    rclpy.init(args=args)
    node = SlamLifecycleAutoStarter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
