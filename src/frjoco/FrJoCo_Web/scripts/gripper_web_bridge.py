#!/usr/bin/env python3
import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64


class GripperWebBridge(Node):
    def __init__(self):
        super().__init__('gripper_web_bridge')
        self.declare_parameter('action_name', '/gripper_controller/gripper_cmd')
        self.declare_parameter('command_topic', '/web/gripper_command')
        self.declare_parameter('max_effort', 50.0)
        self.declare_parameter('server_wait_timeout', 1.0)

        self.action_name = self.get_parameter('action_name').value
        self.command_topic = self.get_parameter('command_topic').value
        self.max_effort = float(self.get_parameter('max_effort').value)
        self.server_wait_timeout = float(self.get_parameter('server_wait_timeout').value)

        self.client = ActionClient(self, GripperCommand, self.action_name)
        self.subscription = self.create_subscription(
            Float64,
            self.command_topic,
            self.command_callback,
            10,
        )
        self.get_logger().info(
            f'Forwarding {self.command_topic} -> {self.action_name} '
            f'(max_effort={self.max_effort:.1f})'
        )

    def command_callback(self, msg):
        if not self.client.wait_for_server(timeout_sec=self.server_wait_timeout):
            self.get_logger().warn(f'Gripper action server is not ready: {self.action_name}')
            return

        goal = GripperCommand.Goal()
        goal.command.position = float(msg.data)
        goal.command.max_effort = self.max_effort
        self.get_logger().info(f'Sending gripper goal position={goal.command.position:.3f}')

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Gripper goal rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f'Gripper result position={result.position:.3f} '
            f'effort={result.effort:.3f} stalled={result.stalled} reached={result.reached_goal}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GripperWebBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
