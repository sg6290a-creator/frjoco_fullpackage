#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PassiveJointStatePublisher(Node):
    def __init__(self):
        super().__init__('passive_joint_state_publisher')
        self.declare_parameter('arm_hardware_enabled', True)
        self.declare_parameter('mobile_hardware_enabled', True)
        self.declare_parameter('gripper_hardware_enabled', True)
        self.declare_parameter('publish_rate', 20.0)

        arm_enabled = bool(self.get_parameter('arm_hardware_enabled').value)
        mobile_enabled = bool(self.get_parameter('mobile_hardware_enabled').value)
        gripper_enabled = bool(self.get_parameter('gripper_hardware_enabled').value)

        self.joint_positions = {}
        if not arm_enabled:
            self.joint_positions.update({
                'shoulder_pan_joint': 0.0,
                'shoulder_lift_joint': 0.0,
                'elbow_joint': 0.0,
                'wrist_1_joint': 0.0,
                'wrist_2_joint': 0.0,
                'wrist_3_joint': 0.0,
            })
        if not mobile_enabled:
            self.joint_positions.update({
                'front_left_wheel_joint': 0.0,
                'front_right_wheel_joint': 0.0,
                'rear_left_wheel_joint': 0.0,
                'rear_right_wheel_joint': 0.0,
            })
        if not gripper_enabled:
            self.joint_positions.update({
                'gripper_left_joint': 0.030,
                'gripper_right_joint': 0.030,
            })

        rate = float(self.get_parameter('publish_rate').value)
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._publish)

        if self.joint_positions:
            names = ', '.join(self.joint_positions.keys())
            self.get_logger().info(f'Publishing passive joint states: {names}')
        else:
            self.get_logger().info('No passive joint states needed.')

    def _publish(self):
        if not self.joint_positions:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.velocity = [0.0] * len(msg.name)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PassiveJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
