#!/usr/bin/env python3
"""
Optional separate keyboard trigger client.

Use this only if you want the key input in a terminal separate from
thin_part_grasp_trigger_node.py.

Run:
  python3 keyboard_trigger_client.py
or after installing as a ROS2 console script.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class KeyboardTriggerClient(Node):
    def __init__(self):
        super().__init__('keyboard_trigger_client')
        self.cli = self.create_client(Trigger, '/thin_part/estimate')
        self.get_logger().info('Waiting for /thin_part/estimate service...')
        self.cli.wait_for_service()
        self.get_logger().info('Ready. Press g + Enter, t + Enter, or just Enter to trigger. q + Enter exits.')

    def send_trigger(self):
        future = self.cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error('Service call failed.')
            return
        res = future.result()
        if res.success:
            self.get_logger().info(res.message)
        else:
            self.get_logger().warn(res.message)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTriggerClient()
    try:
        while rclpy.ok():
            text = input('[trigger-client] g/Enter=estimate, q=quit > ').strip().lower()
            if text in ('q', 'quit', 'exit'):
                break
            if text in ('', 'g', 't', 'trigger'):
                node.send_trigger()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
