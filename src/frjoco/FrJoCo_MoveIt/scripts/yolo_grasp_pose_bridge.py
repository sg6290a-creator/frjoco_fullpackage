#!/usr/bin/env python3
"""
yolo_grasp_pose_bridge.py

YOLO/thin-part grasp output -> PoseStamped(/target_pose) bridge.

Input:
  /hammer_target_point, /pliers_target_point, /screwdriver_target_point
  /thin_part/grasp_info

Trigger:
  /execute_pick_place (std_srvs/Trigger)

Output:
  /target_pose (geometry_msgs/PoseStamped)
  /pick_place_status (std_msgs/String)

This node intentionally does not call MoveIt directly. It publishes a
PoseStamped target that vision_move.py can transform and execute.
"""

import math
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger


CLASS_NAMES = {
    0: 'hammer',
    1: 'pliers',
    2: 'screwdriver',
}


@dataclass
class TargetState:
    point: Optional[PointStamped] = None
    roll_deg: Optional[float] = None
    received_sec: Optional[float] = None


class YoloGraspPoseBridge(Node):
    def __init__(self):
        super().__init__('yolo_grasp_pose_bridge')

        self.declare_parameter('output_topic', '/target_pose')
        self.declare_parameter('execute_service', '/execute_pick_place')
        self.declare_parameter('status_topic', '/pick_place_status')
        self.declare_parameter('camera_frame', 'd405_optical_frame')
        self.declare_parameter('target_object', 'latest')  # latest, hammer, pliers, screwdriver
        self.declare_parameter('max_target_age_sec', 0.0)  # 0 = 제한 없음
        self.declare_parameter('require_roll', True)
        self.declare_parameter('default_roll_deg', 0.0)
        self.declare_parameter('roll_axis', 'z')  # x, y, or z in the input frame
        self.declare_parameter('position_offset_x', 0.0)
        self.declare_parameter('position_offset_y', 0.0)
        self.declare_parameter('position_offset_z', 0.0)
        self.declare_parameter('auto_publish', False)

        self.output_topic = str(self.get_parameter('output_topic').value)
        self.execute_service = str(self.get_parameter('execute_service').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.target_object = str(self.get_parameter('target_object').value).strip().lower()
        self.max_target_age_sec = float(self.get_parameter('max_target_age_sec').value)
        self.require_roll = bool(self.get_parameter('require_roll').value)
        self.default_roll_deg = float(self.get_parameter('default_roll_deg').value)
        self.roll_axis = str(self.get_parameter('roll_axis').value).strip().lower()
        self.position_offset = (
            float(self.get_parameter('position_offset_x').value),
            float(self.get_parameter('position_offset_y').value),
            float(self.get_parameter('position_offset_z').value),
        )
        self.auto_publish = bool(self.get_parameter('auto_publish').value)

        if self.roll_axis not in ('x', 'y', 'z'):
            raise ValueError("roll_axis must be one of: 'x', 'y', 'z'")
        if self.target_object not in ('latest', 'hammer', 'pliers', 'screwdriver'):
            raise ValueError("target_object must be 'latest', 'hammer', 'pliers', or 'screwdriver'")

        self.targets: Dict[int, TargetState] = {cls_id: TargetState() for cls_id in CLASS_NAMES}
        self.latest_class_id: Optional[int] = None

        self.pose_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_subscription(PointStamped, '/hammer_target_point', self._point_cb_factory(0), 10)
        self.create_subscription(PointStamped, '/pliers_target_point', self._point_cb_factory(1), 10)
        self.create_subscription(PointStamped, '/screwdriver_target_point', self._point_cb_factory(2), 10)
        self.create_subscription(Float32MultiArray, '/thin_part/grasp_info', self._grasp_info_cb, 10)

        self.create_service(Trigger, self.execute_service, self._execute_cb)
        for class_id, name in CLASS_NAMES.items():
            self.create_service(
                Trigger,
                f'{self.execute_service}_{name}',
                self._execute_cb_factory(class_id),
            )

        self.get_logger().info(
            f'YOLO grasp pose bridge ready: target_object={self.target_object}, '
            f'output={self.output_topic}, service={self.execute_service}, '
            f'roll_axis={self.roll_axis}, auto_publish={self.auto_publish}'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _point_cb_factory(self, class_id: int):
        def _point_cb(msg: PointStamped):
            state = self.targets[class_id]
            state.point = msg
            state.received_sec = self._now_sec()
            self.latest_class_id = class_id
            name = CLASS_NAMES[class_id]
            frame = msg.header.frame_id or self.camera_frame
            self._publish_status(
                f'YOLO target updated: {name} frame={frame} '
                f'pos=({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})'
            )
            if self.auto_publish and self._target_is_complete(state):
                self._publish_pose_for_class(class_id)

        return _point_cb

    def _grasp_info_cb(self, msg: Float32MultiArray):
        if not msg.data or len(msg.data) < 9:
            return

        class_id = int(round(msg.data[0]))
        if class_id not in self.targets:
            return

        roll_deg = float(msg.data[8])
        state = self.targets[class_id]
        state.roll_deg = roll_deg
        state.received_sec = self._now_sec()
        self.latest_class_id = class_id

        # grasp_info carries x/y/z from the same estimation that produced roll.
        # Keep point and roll synchronized even if the class-specific PointStamped
        # callback arrives before or after this message.
        if len(msg.data) >= 16:
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = self.camera_frame
            point_msg.point.x = float(msg.data[13])
            point_msg.point.y = float(msg.data[14])
            point_msg.point.z = float(msg.data[15])
            state.point = point_msg

        name = CLASS_NAMES[class_id]
        self._publish_status(f'YOLO grasp roll updated: {name} roll={roll_deg:.1f}deg')

        if self.auto_publish and self._target_is_complete(state):
            self._publish_pose_for_class(class_id)

    def _execute_cb(self, request, response):
        class_id = self._select_class_id()
        if class_id is None:
            response.success = False
            response.message = 'No YOLO grasp target has been received.'
            return response

        success, message = self._publish_pose_for_class(class_id)
        response.success = success
        response.message = message
        return response

    def _execute_cb_factory(self, class_id: int):
        def _execute_for_class(request, response):
            success, message = self._publish_pose_for_class(class_id)
            response.success = success
            response.message = message
            return response

        return _execute_for_class

    def _select_class_id(self) -> Optional[int]:
        self.target_object = str(self.get_parameter('target_object').value).strip().lower()
        if self.target_object == 'latest':
            return self.latest_class_id
        for class_id, name in CLASS_NAMES.items():
            if name == self.target_object:
                return class_id
        return None

    def _target_is_complete(self, state: TargetState) -> bool:
        if state.point is None:
            return False
        if self.require_roll and state.roll_deg is None:
            return False
        return True

    def _publish_pose_for_class(self, class_id: int):
        state = self.targets[class_id]
        name = CLASS_NAMES[class_id]

        if state.point is None:
            return False, f'No point available for {name}.'

        if self.require_roll and state.roll_deg is None:
            return False, f'No roll available for {name}; trigger /thin_part/estimate first.'

        if self.max_target_age_sec > 0 and state.received_sec is not None:
            age = self._now_sec() - state.received_sec
            if age > self.max_target_age_sec:
                return False, f'Target for {name} is stale ({age:.1f}s old).'

        roll_deg = state.roll_deg if state.roll_deg is not None else self.default_roll_deg
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = state.point.header.frame_id or self.camera_frame
        pose.pose.position.x = state.point.point.x + self.position_offset[0]
        pose.pose.position.y = state.point.point.y + self.position_offset[1]
        pose.pose.position.z = state.point.point.z + self.position_offset[2]
        pose.pose.orientation = self._roll_to_quaternion(math.radians(roll_deg))

        self.pose_pub.publish(pose)
        message = (
            f'Published MoveIt target_pose for {name}: frame={pose.header.frame_id}, '
            f'pos=({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}), '
            f'roll={roll_deg:.1f}deg'
        )
        self._publish_status(message)
        return True, message

    def _roll_to_quaternion(self, roll_rad: float) -> Quaternion:
        half = roll_rad / 2.0
        s = math.sin(half)
        q = Quaternion()
        q.w = math.cos(half)

        if self.roll_axis == 'x':
            q.x = s
        elif self.roll_axis == 'y':
            q.y = s
        else:
            q.z = s

        return q


def main(args=None):
    rclpy.init(args=args)
    node = YoloGraspPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
