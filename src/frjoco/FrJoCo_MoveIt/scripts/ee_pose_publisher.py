#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class EndEffectorPosePublisher(Node):
    def __init__(self):
        super().__init__('ee_pose_publisher')

        self.declare_parameter('reference_frame', 'base_link')
        self.declare_parameter('left_finger_frame', 'gripper_left_link')
        self.declare_parameter('right_finger_frame', 'gripper_right_link')
        self.declare_parameter('fallback_frame', 'end_effector_link')
        self.declare_parameter('output_topic', '/gripper_center_pose')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('lookup_timeout_sec', 0.02)

        self.reference_frame = str(self.get_parameter('reference_frame').value)
        self.left_finger_frame = str(self.get_parameter('left_finger_frame').value)
        self.right_finger_frame = str(self.get_parameter('right_finger_frame').value)
        self.fallback_frame = str(self.get_parameter('fallback_frame').value)
        output_topic = str(self.get_parameter('output_topic').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.lookup_timeout = Duration(
            seconds=float(self.get_parameter('lookup_timeout_sec').value)
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, output_topic, 10)

        period = 1.0 / max(publish_rate, 0.1)
        self.timer = self.create_timer(period, self._publish_pose)
        self._last_warn_time = 0.0

        self.get_logger().info(
            f'Publishing gripper center pose: {self.reference_frame} <- '
            f'({self.left_finger_frame}, {self.right_finger_frame}), '
            f'fallback={self.fallback_frame}, topic={output_topic}'
        )

    def _lookup(self, source_frame):
        return self.tf_buffer.lookup_transform(
            self.reference_frame,
            source_frame,
            Time(),
            timeout=self.lookup_timeout,
        )

    def _publish_pose(self):
        try:
            left = self._lookup(self.left_finger_frame)
            right = self._lookup(self.right_finger_frame)
            fallback = self._lookup(self.fallback_frame)
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.reference_frame
            pose.pose.position.x = (
                left.transform.translation.x + right.transform.translation.x
            ) * 0.5
            pose.pose.position.y = (
                left.transform.translation.y + right.transform.translation.y
            ) * 0.5
            pose.pose.position.z = (
                left.transform.translation.z + right.transform.translation.z
            ) * 0.5
            pose.pose.orientation = fallback.transform.rotation
            self.pose_pub.publish(pose)
            return
        except TransformException:
            pass

        try:
            fallback = self._lookup(self.fallback_frame)
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.reference_frame
            pose.pose.position.x = fallback.transform.translation.x
            pose.pose.position.y = fallback.transform.translation.y
            pose.pose.position.z = fallback.transform.translation.z
            pose.pose.orientation = fallback.transform.rotation
            self.pose_pub.publish(pose)
        except TransformException as exc:
            now = self.get_clock().now().nanoseconds * 1e-9
            if math.isfinite(now) and now - self._last_warn_time > 5.0:
                self._last_warn_time = now
                self.get_logger().warn(
                    f'Waiting for gripper TF in {self.reference_frame}: {exc}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
