#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')
        self.declare_parameter('robot_description', '')
        self.description = self.get_parameter('robot_description').value

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(String, 'robot_description', qos)

        self.timer = self.create_timer(1.0, self.publish_description)
        self.publish_description()

    def publish_description(self):
        if not self.description:
            self.get_logger().warn('robot_description parameter is empty')
            return

        msg = String()
        msg.data = self.description
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
