#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
import threading
import time


class DemoJointStatePublisher(Node):
    def __init__(self):
        super().__init__('demo_joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
        )
        self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10,
        )
        self.create_subscription(
            Float64,
            '/web/gripper_command',
            self.gripper_callback,
            10,
        )
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.follow_joint_trajectory_callback,
        )
        self.timer = self.create_timer(0.05, self.publish_joint_state)
        self.lock = threading.RLock()

        self.joint_positions = {
            'shoulder_pan_joint': 0.0,
            'shoulder_lift_joint': 0.0,
            'elbow_joint': 0.0,
            'wrist_1_joint': 0.0,
            'wrist_2_joint': 0.0,
            'wrist_3_joint': 0.0,
            'gripper_left_joint': 0.030,
            'gripper_right_joint': 0.030,
            'front_left_wheel_joint': 0.0,
            'front_right_wheel_joint': 0.0,
            'rear_left_wheel_joint': 0.0,
            'rear_right_wheel_joint': 0.0,
        }
        self.get_logger().info(
            'Demo joint state publisher ready: listening to '
            '/joint_trajectory_controller/joint_trajectory, '
            '/joint_trajectory_controller/follow_joint_trajectory, '
            'and /web/gripper_command'
        )

    def trajectory_callback(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn('Ignoring empty JointTrajectory')
            return

        point = msg.points[-1]
        if len(point.positions) < len(msg.joint_names):
            self.get_logger().warn('Ignoring JointTrajectory with too few positions')
            return

        updated = self._apply_joint_positions(msg.joint_names, point.positions)
        if updated:
            self._publish_joint_state_now()
            self._push_planning_scene()
            self.get_logger().info('Sim joint command: ' + ', '.join(updated))

    def follow_joint_trajectory_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        if not trajectory.points:
            self.get_logger().warn('Rejecting empty FollowJointTrajectory goal')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'empty trajectory'
            return result

        if any(len(point.positions) < len(trajectory.joint_names) for point in trajectory.points):
            self.get_logger().warn('Rejecting FollowJointTrajectory goal with too few positions')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'too few positions'
            return result

        if not self._execute_trajectory(goal_handle, trajectory):
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = 'canceled'
            return result

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def _execute_trajectory(self, goal_handle, trajectory):
        joint_names = list(trajectory.joint_names)
        start_positions = [self.joint_positions.get(name, 0.0) for name in joint_names]
        last_time = 0.0

        for index, point in enumerate(trajectory.points):
            target_positions = [float(value) for value in point.positions[:len(joint_names)]]
            target_time = self._duration_to_sec(point.time_from_start)
            segment_time = max(0.0, target_time - last_time)
            steps = max(1, int(segment_time / 0.02))

            for step in range(1, steps + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return False
                alpha = step / steps
                interpolated = [
                    start + (target - start) * alpha
                    for start, target in zip(start_positions, target_positions)
                ]
                self._apply_joint_positions(joint_names, interpolated)
                self._publish_joint_state_now()
                self._publish_feedback(goal_handle, trajectory, index, interpolated)
                if segment_time > 0.0:
                    time.sleep(segment_time / steps)

            start_positions = target_positions
            last_time = target_time

        final_point = trajectory.points[-1]
        updated = self._apply_joint_positions(joint_names, final_point.positions)
        for _ in range(10):
            self._publish_joint_state_now()
            time.sleep(0.02)
        self._push_planning_scene()
        if updated:
            self.get_logger().info('Sim action final state: ' + ', '.join(updated))
        return True

    def _publish_feedback(self, goal_handle, trajectory, point_index, positions):
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(trajectory.joint_names)
        feedback.desired = trajectory.points[point_index]
        feedback.actual.positions = list(positions)
        feedback.actual.time_from_start = feedback.desired.time_from_start
        feedback.error.positions = [
            desired - actual
            for desired, actual in zip(feedback.desired.positions, feedback.actual.positions)
        ]
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _duration_to_sec(duration):
        return float(duration.sec) + float(duration.nanosec) * 1e-9

    def _apply_joint_positions(self, joint_names, positions):
        updated = []
        with self.lock:
            for name, position in zip(joint_names, positions):
                if name not in self.joint_positions:
                    continue
                self.joint_positions[name] = float(position)
                updated.append(f'{name}={position:.3f}')
        return updated

    def gripper_callback(self, msg: Float64):
        position = float(msg.data)
        with self.lock:
            self.joint_positions['gripper_left_joint'] = position
            self.joint_positions['gripper_right_joint'] = position
        self._publish_joint_state_now()
        self._push_planning_scene()
        self.get_logger().info(f'Sim gripper command: {position:.3f}')

    def publish_joint_state(self):
        self._publish_joint_state_now()

    def _publish_joint_state_now(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.name = list(self.joint_positions.keys())
            msg.position = list(self.joint_positions.values())
        self.publisher.publish(msg)

    def _push_planning_scene(self):
        if not self.apply_scene_client.service_is_ready():
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            scene.robot_state.joint_state.name = list(self.joint_positions.keys())
            scene.robot_state.joint_state.position = list(self.joint_positions.values())

        request = ApplyPlanningScene.Request()
        request.scene = scene
        self.apply_scene_client.call_async(request)


def main():
    rclpy.init()
    node = DemoJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.action_server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
