#!/usr/bin/env python3
"""
SLAM Toolbox Lifecycle Auto-Starter
Cleanly transitions slam_toolbox to active and stops immediately once active.
"""

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
        
        self.timer = self.create_timer(0.5, self._check_and_transition)
        self.in_progress = False

    def _check_and_transition(self):
        if self.in_progress:
            return

        if not self.get_state_client.wait_for_service(timeout_sec=0.2):
            return

        self.in_progress = True
        future = self.get_state_client.call_async(GetState.Request())
        future.add_done_callback(self._on_get_state)

    def _on_get_state(self, future):
        try:
            state = future.result().current_state.label
            if state == 'active':
                self.get_logger().info('✅ SLAM Toolbox is ACTIVE! Occupancy grid /map and TF map->odom are live.')
                self.timer.cancel()
                self.in_progress = False
                return

            if state == 'unconfigured':
                self.get_logger().info('SLAM Toolbox is UNCONFIGURED. Sending TRANSITION_CONFIGURE...')
                req = ChangeState.Request()
                req.transition.id = Transition.TRANSITION_CONFIGURE
                fut = self.change_state_client.call_async(req)
                fut.add_done_callback(lambda f: self._set_in_progress(False))
                return

            if state == 'inactive':
                self.get_logger().info('SLAM Toolbox is INACTIVE. Sending TRANSITION_ACTIVATE...')
                req = ChangeState.Request()
                req.transition.id = Transition.TRANSITION_ACTIVATE
                fut = self.change_state_client.call_async(req)
                fut.add_done_callback(lambda f: self._set_in_progress(False))
                return

        except Exception as e:
            self.get_logger().error(f'State query error: {e}')
        
        self.in_progress = False

    def _set_in_progress(self, val):
        self.in_progress = val


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
