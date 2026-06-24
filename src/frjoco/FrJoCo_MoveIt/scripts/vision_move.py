#!/usr/bin/env python3
"""
vision_move.py
PoseStamped 수신 → Look-at 방향 계산 → IK → Plan → RViz 시각화 → 사용자 확인 → Execute

파라미터:
  planning_group / planning_frame / tip_link
  position_tolerance / orientation_tolerance
  planning_time / vel_scale / acc_scale
  require_confirmation : True
  input_topic          : '/target_pose'
  use_input_orientation: False
"""

import math
import copy
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float32MultiArray, String
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, MoveItErrorCodes, JointConstraint,
    PositionConstraint, OrientationConstraint, BoundingVolume,
    DisplayTrajectory, PlanningScene,
)
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401


class VisionMove(Node):
    def __init__(self):
        super().__init__('vision_move')

        self.declare_parameter('planning_group',        'arm')
        self.declare_parameter('planning_frame',        'base_link')
        self.declare_parameter('tip_link',              'tool0')
        self.declare_parameter('planning_time',         5.0)
        self.declare_parameter('vel_scale',             0.3)
        self.declare_parameter('acc_scale',             0.3)
        self.declare_parameter('position_tolerance',    0.01)
        self.declare_parameter('orientation_tolerance', 0.10)
        self.declare_parameter('planning_mode',         'manual_ik')  # manual_ik, moveit_pose
        self.declare_parameter('require_confirmation',  True)
        self.declare_parameter('input_topic',           '/target_pose')
        self.declare_parameter('execute_topic',         '/execute_target_pose')
        self.declare_parameter('home_on_start',         False)
        self.declare_parameter('use_input_orientation', False)
        self.declare_parameter('generated_orientation_mode', 'current')  # current, look_at
        self.declare_parameter('current_state_timeout', 20.0)
        self.declare_parameter('workspace_gate_enabled', True)
        self.declare_parameter('workspace_min_x', -0.20)
        self.declare_parameter('workspace_max_x', 0.80)
        self.declare_parameter('workspace_min_y', -0.55)
        self.declare_parameter('workspace_max_y', 0.55)
        self.declare_parameter('workspace_min_z', 0.02)
        self.declare_parameter('workspace_max_z', 0.90)
        self.declare_parameter('workspace_min_xy_radius', 0.05)
        self.declare_parameter('workspace_max_xy_radius', 0.85)
        self.declare_parameter('run_grasp_sequence', False)
        self.declare_parameter('status_topic', '/pick_place_status')
        self.declare_parameter('open_before_grasp', True)
        self.declare_parameter('pre_grasp_align_enabled', True)
        self.declare_parameter('pre_grasp_align_x_offset', -0.10)
        self.declare_parameter('pre_grasp_align_settle_sec', 0.7)
        self.declare_parameter('force_open_before_close', True)
        self.declare_parameter('hold_manual_gripper_open', True)
        self.declare_parameter('gripper_hold_joint_name', 'gripper_left_joint')
        self.declare_parameter('gripper_open_position', 0.029)
        self.declare_parameter('gripper_close_position', 0.020)
        self.declare_parameter('gripper_max_effort', 50.0)
        self.declare_parameter('gripper_action', '/gripper_controller/gripper_cmd')
        self.declare_parameter('grasp_roll_enabled', True)
        self.declare_parameter('grasp_info_topic', '/thin_part/grasp_info')
        self.declare_parameter('grasp_roll_joint_name', 'wrist_3_joint')
        self.declare_parameter('grasp_roll_sign', 1.0)
        self.declare_parameter('grasp_roll_offset_deg', 0.0)
        self.declare_parameter('grasp_roll_max_abs_deg', 90.0)
        self.declare_parameter('grasp_roll_max_age_sec', 5.0)
        self.declare_parameter('place_transition_mode', 'direct')  # direct, load_turn
        self.declare_parameter('load_turn_joint_name', 'shoulder_pan_joint')
        self.declare_parameter('load_turn_angle', 1.5708)
        self.declare_parameter('carry_elbow_extra_bend_deg', 6.0)
        self.declare_parameter('post_place_turn_enabled', True)
        self.declare_parameter('post_place_turn_joint_name', 'shoulder_pan_joint')
        self.declare_parameter('post_place_turn_angle', -3.1416)
        self.declare_parameter('pre_open_after_place_delay_sec', 1.5)
        self.declare_parameter('place_shoulder_pan_joint', self._PLACE_JOINTS['shoulder_pan_joint'])
        self.declare_parameter('place_shoulder_lift_joint', self._PLACE_JOINTS['shoulder_lift_joint'])
        self.declare_parameter('place_elbow_joint', self._PLACE_JOINTS['elbow_joint'])
        self.declare_parameter('place_wrist_1_joint', self._PLACE_JOINTS['wrist_1_joint'])
        self.declare_parameter('place_wrist_2_joint', self._PLACE_JOINTS['wrist_2_joint'])
        self.declare_parameter('place_wrist_3_joint', self._PLACE_JOINTS['wrist_3_joint'])
        self.declare_parameter('open_after_place', True)
        self.declare_parameter('return_ready_after_place', True)
        self.declare_parameter('gripper_settle_sec', 1.0)
        self.declare_parameter('post_grasp_settle_sec', 0.5)
        self.declare_parameter('clear_rviz_after_execute', False)
        self.declare_parameter('execute_after_plan', True)
        self.declare_parameter('identity_orientation_is_missing', False)

        self.planning_group       = self.get_parameter('planning_group').value
        self.planning_frame       = self.get_parameter('planning_frame').value
        self.tip_link             = self.get_parameter('tip_link').value
        self.planning_time        = self.get_parameter('planning_time').value
        self.vel_scale            = self.get_parameter('vel_scale').value
        self.acc_scale            = self.get_parameter('acc_scale').value
        self.position_tolerance   = float(self.get_parameter('position_tolerance').value)
        self.orientation_tolerance = float(self.get_parameter('orientation_tolerance').value)
        self.planning_mode        = str(self.get_parameter('planning_mode').value).strip().lower()
        self.require_confirmation = self.get_parameter('require_confirmation').value
        input_topic               = self.get_parameter('input_topic').value
        execute_topic             = self.get_parameter('execute_topic').value
        home_on_start             = self.get_parameter('home_on_start').value
        self.use_input_orientation = bool(self.get_parameter('use_input_orientation').value)
        self.generated_orientation_mode = str(
            self.get_parameter('generated_orientation_mode').value).strip().lower()
        self.current_state_timeout = float(self.get_parameter('current_state_timeout').value)
        self.workspace_gate_enabled = bool(
            self.get_parameter('workspace_gate_enabled').value)
        self.workspace_min_x = float(self.get_parameter('workspace_min_x').value)
        self.workspace_max_x = float(self.get_parameter('workspace_max_x').value)
        self.workspace_min_y = float(self.get_parameter('workspace_min_y').value)
        self.workspace_max_y = float(self.get_parameter('workspace_max_y').value)
        self.workspace_min_z = float(self.get_parameter('workspace_min_z').value)
        self.workspace_max_z = float(self.get_parameter('workspace_max_z').value)
        self.workspace_min_xy_radius = float(
            self.get_parameter('workspace_min_xy_radius').value)
        self.workspace_max_xy_radius = float(
            self.get_parameter('workspace_max_xy_radius').value)
        self.run_grasp_sequence = bool(self.get_parameter('run_grasp_sequence').value)
        status_topic = self.get_parameter('status_topic').value
        self.open_before_grasp = bool(self.get_parameter('open_before_grasp').value)
        self.pre_grasp_align_enabled = bool(
            self.get_parameter('pre_grasp_align_enabled').value)
        self.pre_grasp_align_x_offset = float(
            self.get_parameter('pre_grasp_align_x_offset').value)
        self.pre_grasp_align_settle_sec = max(0.0, float(
            self.get_parameter('pre_grasp_align_settle_sec').value))
        self.force_open_before_close = bool(
            self.get_parameter('force_open_before_close').value)
        self.hold_manual_gripper_open = bool(
            self.get_parameter('hold_manual_gripper_open').value)
        self.gripper_hold_joint_name = self.get_parameter('gripper_hold_joint_name').value
        self.gripper_open_position = float(self.get_parameter('gripper_open_position').value)
        self.gripper_close_position = float(self.get_parameter('gripper_close_position').value)
        self.gripper_max_effort = float(self.get_parameter('gripper_max_effort').value)
        gripper_action = self.get_parameter('gripper_action').value
        self.grasp_roll_enabled = bool(self.get_parameter('grasp_roll_enabled').value)
        grasp_info_topic = self.get_parameter('grasp_info_topic').value
        self.grasp_roll_joint_name = self.get_parameter('grasp_roll_joint_name').value
        self.grasp_roll_sign = float(self.get_parameter('grasp_roll_sign').value)
        self.grasp_roll_offset_deg = float(self.get_parameter('grasp_roll_offset_deg').value)
        self.grasp_roll_max_abs_deg = abs(float(
            self.get_parameter('grasp_roll_max_abs_deg').value))
        self.grasp_roll_max_age_sec = float(
            self.get_parameter('grasp_roll_max_age_sec').value)
        self.place_transition_mode = str(
            self.get_parameter('place_transition_mode').value).strip().lower()
        self.load_turn_joint_name = self.get_parameter('load_turn_joint_name').value
        self.load_turn_angle = float(self.get_parameter('load_turn_angle').value)
        self.carry_elbow_extra_bend_deg = float(
            self.get_parameter('carry_elbow_extra_bend_deg').value)
        self.post_place_turn_enabled = bool(
            self.get_parameter('post_place_turn_enabled').value)
        self.post_place_turn_joint_name = self.get_parameter(
            'post_place_turn_joint_name').value
        self.post_place_turn_angle = float(
            self.get_parameter('post_place_turn_angle').value)
        self.pre_open_after_place_delay_sec = max(0.0, float(
            self.get_parameter('pre_open_after_place_delay_sec').value))
        self.place_joints = {
            'shoulder_pan_joint': float(self.get_parameter('place_shoulder_pan_joint').value),
            'shoulder_lift_joint': float(self.get_parameter('place_shoulder_lift_joint').value),
            'elbow_joint': float(self.get_parameter('place_elbow_joint').value),
            'wrist_1_joint': float(self.get_parameter('place_wrist_1_joint').value),
            'wrist_2_joint': float(self.get_parameter('place_wrist_2_joint').value),
            'wrist_3_joint': float(self.get_parameter('place_wrist_3_joint').value),
        }
        self.open_after_place = bool(self.get_parameter('open_after_place').value)
        self.return_ready_after_place = bool(self.get_parameter('return_ready_after_place').value)
        self.gripper_settle_sec = float(self.get_parameter('gripper_settle_sec').value)
        self.post_grasp_settle_sec = float(self.get_parameter('post_grasp_settle_sec').value)
        self.clear_rviz_after_execute = bool(
            self.get_parameter('clear_rviz_after_execute').value)
        self.execute_after_plan = bool(self.get_parameter('execute_after_plan').value)
        self.identity_orientation_is_missing = bool(
            self.get_parameter('identity_orientation_is_missing').value)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cb_group = ReentrantCallbackGroup()

        self.plan_client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self.cb_group)
        self.exec_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory', callback_group=self.cb_group)
        self.gripper_client = ActionClient(
            self, GripperCommand, gripper_action, callback_group=self.cb_group)
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.cb_group)
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=self.cb_group)
        self.display_pub = self.create_publisher(
            DisplayTrajectory, '/display_planned_path', 1)
        self.move_group_display_pub = self.create_publisher(
            DisplayTrajectory, '/move_group/display_planned_path', 1)
        self.goal_marker_pub = self.create_publisher(
            Marker, '/vision_move/goal_marker', 1)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        self.get_logger().info('Waiting for /move_action ...')
        if not self.plan_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError('MoveGroup not ready')

        if self.execute_after_plan:
            self.get_logger().info('Waiting for /execute_trajectory ...')
            if not self.exec_client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError('ExecuteTrajectory not ready')

        if self.planning_mode not in ('moveit_pose', 'manual_ik'):
            raise ValueError("planning_mode must be 'moveit_pose' or 'manual_ik'")
        if self.generated_orientation_mode not in ('current', 'look_at'):
            raise ValueError("generated_orientation_mode must be 'current' or 'look_at'")
        if self.workspace_min_x > self.workspace_max_x:
            raise ValueError('workspace_min_x must be <= workspace_max_x')
        if self.workspace_min_y > self.workspace_max_y:
            raise ValueError('workspace_min_y must be <= workspace_max_y')
        if self.workspace_min_z > self.workspace_max_z:
            raise ValueError('workspace_min_z must be <= workspace_max_z')
        if self.workspace_min_xy_radius > self.workspace_max_xy_radius:
            raise ValueError('workspace_min_xy_radius must be <= workspace_max_xy_radius')

        if self.planning_mode == 'manual_ik':
            self.get_logger().info('Waiting for /compute_ik ...')
            if not self.ik_client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('/compute_ik not ready')

        if self.run_grasp_sequence:
            self.get_logger().info(f'Waiting for {gripper_action} ...')
            if not self.gripper_client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError(f'{gripper_action} not ready')

        self.busy = False
        self._pending_trajectory = None
        self._pending_pose = None
        self._js: JointState | None = None
        self._js_lock = threading.Lock()
        self._latest_grasp_roll_deg: float | None = None
        self._latest_grasp_roll_stamp: float | None = None
        self._grasp_roll_lock = threading.Lock()
        self._manual_gripper_open_position: float | None = (
            self.gripper_open_position if self.hold_manual_gripper_open else None)
        if self._manual_gripper_open_position is not None:
            self.get_logger().info(
                f'launch gripper open 기준 고정: '
                f'{self.gripper_hold_joint_name}={self._manual_gripper_open_position:.4f}')
        self._arm_joint_names = set(self._HOME_JOINTS.keys())

        self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            PoseStamped, input_topic, self._target_cb, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            Empty, execute_topic, self._execute_pending_cb, 1,
            callback_group=self.cb_group)
        self.create_subscription(
            Empty, '/go_home', self._go_home_cb, 1,
            callback_group=self.cb_group)
        self.create_subscription(
            Float32MultiArray, grasp_info_topic, self._grasp_info_cb, 10,
            callback_group=self.cb_group)

        mode = '확인 후 실행' if self.require_confirmation else '즉시 실행'
        if self.use_input_orientation:
            orient_mode = '입력 orientation 사용'
        elif self.generated_orientation_mode == 'current':
            orient_mode = '현재 EE orientation 유지'
        else:
            orient_mode = 'look-at orientation 자동 계산'
        self.get_logger().info(
            f'VisionMove 시작 — group={self.planning_group} '
            f'frame={self.planning_frame} tip={self.tip_link} '
            f'topic={input_topic} execute_topic={execute_topic} '
            f'planning_mode={self.planning_mode} '
            f'[{mode}, {orient_mode}]')
        if self.workspace_gate_enabled:
            self.get_logger().info(
                'Workspace gate enabled in planning frame '
                f'{self.planning_frame}: '
                f'x=[{self.workspace_min_x:.2f},{self.workspace_max_x:.2f}], '
                f'y=[{self.workspace_min_y:.2f},{self.workspace_max_y:.2f}], '
                f'z=[{self.workspace_min_z:.2f},{self.workspace_max_z:.2f}], '
                f'xy_r=[{self.workspace_min_xy_radius:.2f},{self.workspace_max_xy_radius:.2f}]')
        if self.run_grasp_sequence:
            self.get_logger().info(
                'Grasp sequence enabled — target 이동 후 '
                'manual-open hold → wrist roll → manual-open hold → close → ready → place → open → ready 순서로 실행')
            if self.grasp_roll_enabled:
                self.get_logger().info(
                    f'Grasp roll enabled — {self.grasp_roll_joint_name} '
                    f'uses /thin_part/grasp_info angle with sign={self.grasp_roll_sign:.2f}, '
                    f'offset={self.grasp_roll_offset_deg:.1f}deg, '
                    f'limit=±{self.grasp_roll_max_abs_deg:.1f}deg')
            if self.place_transition_mode == 'load_turn':
                self.get_logger().info(
                    f'Place transition: ready → {self.load_turn_joint_name} '
                    f'+{self.load_turn_angle:.3f}rad → place')
            elif self.place_transition_mode != 'direct':
                raise ValueError("place_transition_mode must be 'direct' or 'load_turn'")
            if self.post_place_turn_enabled:
                self.get_logger().info(
                    f'Post-place turn: place → {self.post_place_turn_joint_name} '
                    f'{self.post_place_turn_angle:+.3f}rad → open → place → ready')

        if home_on_start:
            self._home_timer = self.create_timer(
                1.0, self._home_on_start_cb, callback_group=self.cb_group)

    _HOME_JOINTS = {
        'shoulder_pan_joint':  0.0,
        'shoulder_lift_joint':  0.0,
        'elbow_joint':          0.0,
        'wrist_1_joint':        0.0,
        'wrist_2_joint':        0.0,
        'wrist_3_joint':        0.0,
    }

    _READY_JOINTS = {
        'shoulder_pan_joint':  0.0,
        'shoulder_lift_joint':  0.5901,
        'elbow_joint':         -1.9266,
        'wrist_1_joint':        1.3538,
        'wrist_2_joint':        0.0,
        'wrist_3_joint':        0.0,
    }

    _PLACE_JOINTS = {
        'shoulder_pan_joint':   0.0,
        'shoulder_lift_joint': -0.0700,
        'elbow_joint':         -2.4870,
        'wrist_1_joint':        1.3538,
        'wrist_2_joint':        0.0,
        'wrist_3_joint':        0.0,
    }

    _JOINT_POSITION_LIMITS = {
        'shoulder_pan_joint':  (-math.pi, math.pi),
        'shoulder_lift_joint': (-2.0 * math.pi, 2.0 * math.pi),
        'elbow_joint':         (-math.pi, math.pi),
        'wrist_1_joint':       (-2.0 * math.pi, 2.0 * math.pi),
        'wrist_2_joint':       (-2.0 * math.pi, 2.0 * math.pi),
        'wrist_3_joint':       (-2.0 * math.pi, 2.0 * math.pi),
    }

    # ── /joint_states 추적 ────────────────────────────────────────────────────
    def _js_cb(self, msg: JointState):
        with self._js_lock:
            self._js = msg
        self._capture_manual_gripper_open_from_joint_state(msg)

    def _capture_manual_gripper_open_from_joint_state(self, msg: JointState):
        if not self.hold_manual_gripper_open:
            return
        if self._manual_gripper_open_position is not None:
            return
        for name, position in zip(msg.name, msg.position):
            if name == self.gripper_hold_joint_name:
                self._manual_gripper_open_position = float(position)
                self._publish_status(
                    f'수동 gripper open 기준 저장: {name}={position:.4f}')
                return

    def _grasp_info_cb(self, msg: Float32MultiArray):
        if not msg.data or len(msg.data) < 9:
            return
        roll_deg = float(msg.data[8])
        with self._grasp_roll_lock:
            self._latest_grasp_roll_deg = roll_deg
            self._latest_grasp_roll_stamp = time.monotonic()

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _joint_state_ready(self) -> bool:
        with self._js_lock:
            js = self._js
        if js is None:
            return False
        if js.header.stamp.sec == 0 and js.header.stamp.nanosec == 0:
            return False
        return self._arm_joint_names.issubset(set(js.name))

    def _wait_for_current_state(self, reason: str) -> bool:
        deadline = time.monotonic() + self.current_state_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._joint_state_ready():
                return True
            time.sleep(0.1)
        self.get_logger().error(
            f'{reason} 실패: /joint_states 최신 관절값을 받지 못함 '
            f'({self.current_state_timeout:.1f}s 대기)')
        return False

    def _current_arm_positions(self) -> dict:
        with self._js_lock:
            js = self._js
        if js is None:
            return {}
        return {
            name: position
            for name, position in zip(js.name, js.position)
            if name in self._arm_joint_names
        }

    def _carry_joints(self) -> dict:
        joints = dict(self._READY_JOINTS)
        joints['elbow_joint'] -= math.radians(self.carry_elbow_extra_bend_deg)
        return self._normalize_joint_targets(joints)

    def _pre_grasp_align_pose(self):
        if self._pending_pose is None:
            return None
        pose = copy.deepcopy(self._pending_pose)
        pose.position.x += self.pre_grasp_align_x_offset
        return pose

    def _check_workspace_gate(self, pose: Pose) -> bool:
        if not self.workspace_gate_enabled:
            return True

        p = pose.position
        values = (p.x, p.y, p.z)
        if not all(math.isfinite(v) for v in values):
            self._publish_status('목표 거부: 좌표에 NaN/Inf 포함')
            return False

        xy_radius = math.hypot(p.x, p.y)
        violations = []
        if not (self.workspace_min_x <= p.x <= self.workspace_max_x):
            violations.append(
                f'x={p.x:.3f} not in [{self.workspace_min_x:.3f},{self.workspace_max_x:.3f}]')
        if not (self.workspace_min_y <= p.y <= self.workspace_max_y):
            violations.append(
                f'y={p.y:.3f} not in [{self.workspace_min_y:.3f},{self.workspace_max_y:.3f}]')
        if not (self.workspace_min_z <= p.z <= self.workspace_max_z):
            violations.append(
                f'z={p.z:.3f} not in [{self.workspace_min_z:.3f},{self.workspace_max_z:.3f}]')
        if not (self.workspace_min_xy_radius <= xy_radius <= self.workspace_max_xy_radius):
            violations.append(
                f'xy_r={xy_radius:.3f} not in '
                f'[{self.workspace_min_xy_radius:.3f},{self.workspace_max_xy_radius:.3f}]')

        if violations:
            reason = '; '.join(violations)
            self.get_logger().warn(f'Workspace gate rejected target: {reason}')
            self._publish_status(f'목표 거부: workspace gate ({reason})')
            return False
        return True

    @staticmethod
    def _nearest_equivalent_angle(target: float, reference: float) -> float:
        delta = math.atan2(math.sin(target - reference), math.cos(target - reference))
        return reference + delta

    def _normalize_joint_targets(self, joint_positions: dict) -> dict:
        current = self._current_arm_positions()
        normalized = {}
        adjusted = []

        for name, raw_value in joint_positions.items():
            value = float(raw_value)
            if name in current:
                value = self._nearest_equivalent_angle(value, current[name])

            if name in self._JOINT_POSITION_LIMITS:
                lower, upper = self._JOINT_POSITION_LIMITS[name]
                value = min(max(value, lower), upper)

            normalized[name] = value
            if abs(value - float(raw_value)) > 1e-3:
                adjusted.append(f'{name}: {float(raw_value):.3f}->{value:.3f}')

        if adjusted:
            self.get_logger().info('IK joint wrap 보정: ' + ', '.join(adjusted))
        return normalized

    def _home_on_start_cb(self):
        self._home_timer.cancel()
        self._go_home_cb(None)

    def _go_home_cb(self, _):
        if self.busy:
            self.get_logger().warn('이전 동작 처리 중, 홈 요청 무시됨')
            return
        self.busy = True
        try:
            self.get_logger().info('홈 포즈로 이동...')
            if not self._wait_for_current_state('홈 이동 준비'):
                return
            trajectory = self._plan_joints(self._HOME_JOINTS)
            if trajectory:
                self._execute(trajectory)
                self._clear_display_after_execute()
                self._sync_planning_scene()
        finally:
            self.busy = False

    def _clear_display(self):
        """RViz 계획 경로 시각화 초기화."""
        empty = DisplayTrajectory()
        self.display_pub.publish(empty)
        self.move_group_display_pub.publish(empty)
        m = Marker()
        m.header.frame_id = self.planning_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'vision_move'
        m.action = Marker.DELETE
        m.id = 0
        self.goal_marker_pub.publish(m)

    def _clear_display_after_execute(self):
        if self.clear_rviz_after_execute:
            self._clear_display()

    def _publish_goal_marker(self, pose: Pose):
        """목표 EE 위치를 빨간 구체로 RViz에 표시."""
        m = Marker()
        m.header.frame_id = self.planning_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'vision_move'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = pose.position.x
        m.pose.position.y = pose.position.y
        m.pose.position.z = pose.position.z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.05
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 0.8
        self.goal_marker_pub.publish(m)

    def _publish_display_trajectory(self, trajectory, start_state):
        msg = DisplayTrajectory()
        msg.trajectory_start = start_state
        msg.trajectory.append(trajectory)
        self.display_pub.publish(msg)
        self.move_group_display_pub.publish(msg)

    def _sync_planning_scene(self):
        """실행 후 현재 관절 상태를 planning scene에 강제 반영 → RViz 갱신."""
        with self._js_lock:
            js = self._js
        if js is None:
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.joint_state = js
        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self.apply_scene_client.call_async(req)
        self._await(fut, timeout=3.0)

    # ── 유틸: threading.Event 로 Future 대기 (콜백 내 안전) ──────────────────
    @staticmethod
    def _await(future, timeout: float) -> bool:
        """future 완료까지 대기. 완료되면 True, 타임아웃이면 False."""
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        return event.wait(timeout=timeout)

    # ── TF ───────────────────────────────────────────────────────────────────
    def _current_ee_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.planning_frame, self.tip_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
        except TransformException as e:
            self.get_logger().error(f'TF 조회 실패: {e}')
            return None

    def _current_ee_position(self):
        tf = self._current_ee_transform()
        if tf is None:
            return None
        return tf.transform.translation

    def _transform_pose(self, msg: PoseStamped, target_frame: str):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            return tf2_geometry_msgs.do_transform_pose_stamped(msg, tf)
        except TransformException as e:
            self.get_logger().error(f'TF 변환 실패: {e}')
            return None

    # ── Look-at 쿼터니언 ──────────────────────────────────────────────────────
    @staticmethod
    def _look_at_quaternion(ee_pos, target_pos):
        """tool0 +Z 축이 ee_pos → target_pos 방향을 가리키는 쿼터니언."""
        dx = target_pos.x - ee_pos.x
        dy = target_pos.y - ee_pos.y
        dz = target_pos.z - ee_pos.z
        length = math.sqrt(dx*dx + dy*dy + dz*dz)
        if length < 1e-4:
            return None
        dx /= length; dy /= length; dz /= length

        ax, ay = -dy, dx          # cross([0,0,1], [dx,dy,dz]) = [-dy, dx, 0]
        sin_a = math.sqrt(ax*ax + ay*ay)
        cos_a = dz

        q = Quaternion()
        if sin_a < 1e-6:
            if cos_a > 0:
                q.w = 1.0
            else:
                q.x = 1.0         # 180° X축 회전
        else:
            ax /= sin_a; ay /= sin_a
            h = math.atan2(sin_a, cos_a) / 2.0
            s = math.sin(h)
            q.w = math.cos(h); q.x = ax * s; q.y = ay * s; q.z = 0.0
        return q

    # ── 수신 콜백 ─────────────────────────────────────────────────────────────
    def _target_cb(self, msg: PoseStamped):
        if self.busy:
            self.get_logger().warn('이전 동작 처리 중, 무시됨')
            return
        self.busy = True
        try:
            if msg.header.frame_id and msg.header.frame_id != self.planning_frame:
                msg = self._transform_pose(msg, self.planning_frame)
                if msg is None:
                    return

            if not self._check_workspace_gate(msg.pose):
                return

            if not self._wait_for_current_state('목표 이동 준비'):
                return

            use_supplied_orientation = (
                self.use_input_orientation
                and self._orientation_is_valid(msg.pose.orientation)
                and not (
                    self.identity_orientation_is_missing
                    and self._orientation_is_identity(msg.pose.orientation)
                )
            )

            if not use_supplied_orientation:
                if self.use_input_orientation:
                    self.get_logger().info(
                        '입력 orientation이 없거나 identity라서 look-at orientation으로 대체')
                ee_tf = self._current_ee_transform()
                if ee_tf is None:
                    self.get_logger().error('현재 EE TF 조회 실패')
                    return

                if self.generated_orientation_mode == 'current':
                    q = ee_tf.transform.rotation
                else:
                    q = self._look_at_quaternion(
                        ee_tf.transform.translation, msg.pose.position)
                    if q is None:
                        self.get_logger().warn('목표가 현재 EE와 너무 가까움')
                        return
                msg.pose.orientation = q
            p = msg.pose.position
            self._publish_status(
                f'목표 preview pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) | orientation 준비 완료')

            trajectory = self._plan(msg.pose)
            if trajectory is None:
                return

            self._pending_trajectory = trajectory
            self._pending_pose = msg.pose
            self._publish_status('목표 계획 완료: RViz에서 확인 후 Pick & Place로 실행')
            self.get_logger().info(
                '목표 계획을 저장했습니다. /execute_target_pose 수신 시 실행합니다.')
        finally:
            self.busy = False

    def _execute_pending_cb(self, _msg: Empty):
        if self.busy:
            self.get_logger().warn('이전 동작 처리 중, 실행 요청 무시됨')
            return
        self.busy = True
        try:
            if self._pending_trajectory is None:
                self.get_logger().warn('실행할 목표 계획이 없습니다. 먼저 목표 전송으로 plan을 만드세요.')
                self._publish_status('실행 실패: 저장된 목표 계획 없음')
                return

            if self.run_grasp_sequence and self.open_before_grasp:
                self._publish_status('[1/11] 시작 전 수동 open 위치 고정')
                if not self._hold_manual_gripper_open():
                    return
                time.sleep(self.gripper_settle_sec)

            final_trajectory = self._pending_trajectory
            if (self.run_grasp_sequence
                    and self.pre_grasp_align_enabled
                    and abs(self.pre_grasp_align_x_offset) > 1e-9):
                align_pose = self._pre_grasp_align_pose()
                if align_pose is None:
                    self._publish_status('정렬 실패: 저장된 목표 pose 없음')
                    return
                if not self._check_workspace_gate(align_pose):
                    return
                self._publish_status(
                    f'목표 전 정렬 pose 이동: target x {self.pre_grasp_align_x_offset:+.3f}m')
                align_trajectory = self._plan(align_pose)
                if align_trajectory is None or not self._execute(align_trajectory):
                    self.get_logger().error('pre-grasp 정렬 pose 이동 실패')
                    return
                self._clear_display_after_execute()
                self._sync_planning_scene()
                time.sleep(self.pre_grasp_align_settle_sec)
                if not self._wait_for_current_state('pre-grasp 정렬 후 final target 재계획'):
                    return
                if self._pending_pose is None:
                    self._publish_status('final target 재계획 실패: 저장된 목표 pose 없음')
                    return
                self._publish_status('정렬 후 현재 상태 기준 final target 재계획')
                final_trajectory = self._plan(self._pending_pose)
                if final_trajectory is None:
                    self.get_logger().error('final target 재계획 실패')
                    return

            self._publish_status('저장된 목표 계획 실행 시작')
            if not self._execute(final_trajectory):
                return
            self._clear_display_after_execute()
            self._sync_planning_scene()

            self._pending_trajectory = None
            self._pending_pose = None

            if self.run_grasp_sequence:
                self._run_post_grasp_sequence()
        finally:
            self.busy = False

    # ── IK ───────────────────────────────────────────────────────────────────
    def _compute_ik(self, pose: Pose):
        req = GetPositionIK.Request()
        req.ik_request.group_name       = self.planning_group
        req.ik_request.ik_link_name     = self.tip_link
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec      = 2

        ps = PoseStamped()
        ps.header.frame_id = self.planning_frame
        ps.pose = pose
        req.ik_request.pose_stamped = ps
        with self._js_lock:
            js = self._js
        if js is not None:
            req.ik_request.robot_state.is_diff = True
            req.ik_request.robot_state.joint_state = js

        fut = self.ik_client.call_async(req)
        if not self._await(fut, timeout=6.0):
            self.get_logger().error('IK 타임아웃')
            return None

        resp = fut.result()
        if resp is None or resp.error_code.val != MoveItErrorCodes.SUCCESS:
            code = resp.error_code.val if resp else -1
            self.get_logger().error(f'IK 실패 (code={code}) — 해당 위치/방향 도달 불가')
            return None

        arm_joints = {
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        }
        js = resp.solution.joint_state
        return {n: p for n, p in zip(js.name, js.position) if n in arm_joints}

    @staticmethod
    def _orientation_is_valid(q: Quaternion) -> bool:
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        return norm > 1e-6

    @staticmethod
    def _orientation_is_identity(q: Quaternion) -> bool:
        return (
            abs(q.x) < 1e-6
            and abs(q.y) < 1e-6
            and abs(q.z) < 1e-6
            and abs(q.w - 1.0) < 1e-6
        )

    # ── 계획 ─────────────────────────────────────────────────────────────────
    def _plan(self, pose: Pose):
        self._publish_goal_marker(pose)
        if self.planning_mode == 'moveit_pose':
            self.get_logger().info('MoveIt pose goal 경로 계획 중...')
            return self._plan_pose_goal(pose)

        self.get_logger().info('IK 계산 중...')
        joints = self._compute_ik(pose)
        if joints is None:
            return None
        joints = self._normalize_joint_targets(joints)
        self.get_logger().info('경로 계획 중...')
        return self._plan_joints(joints)

    def _make_motion_plan_request(self, constraints: Constraints) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name                      = self.planning_group
        req.num_planning_attempts           = 5
        req.allowed_planning_time           = self.planning_time
        req.max_velocity_scaling_factor     = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        req.goal_constraints.append(constraints)

        # 이동 후 planning scene 갱신 지연을 우회: 실제 joint state를 직접 주입
        with self._js_lock:
            js = self._js
        if js is not None:
            req.start_state.is_diff = True
            req.start_state.joint_state = js
        return req

    def _send_motion_plan_request(self, req: MotionPlanRequest):
        goal = MoveGroup.Goal()
        goal.request                    = req
        goal.planning_options.plan_only = True

        fut_gh = self.plan_client.send_goal_async(goal)
        if not self._await(fut_gh, timeout=10.0):
            self.get_logger().error('계획 goal 전송 타임아웃')
            return None
        gh = fut_gh.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('계획 goal 거부됨')
            return None

        fut_res = gh.get_result_async()
        if not self._await(fut_res, timeout=30.0):
            self.get_logger().error('계획 타임아웃')
            return None
        res = fut_res.result()
        if res is None or res.result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = res.result.error_code.val if res else -1
            self.get_logger().error(f'계획 실패 (code={code})')
            return None

        self._publish_display_trajectory(res.result.planned_trajectory, req.start_state)
        self.get_logger().info('계획 완료 — RViz에서 경로를 확인하세요')
        return res.result.planned_trajectory

    def _plan_pose_goal(self, pose: Pose):
        constraints = Constraints()

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [self.position_tolerance]

        region_pose = Pose()
        region_pose.position = pose.position
        region_pose.orientation.w = 1.0

        volume = BoundingVolume()
        volume.primitives.append(primitive)
        volume.primitive_poses.append(region_pose)

        pc = PositionConstraint()
        pc.header.frame_id = self.planning_frame
        pc.link_name = self.tip_link
        pc.constraint_region = volume
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = self.planning_frame
        oc.link_name = self.tip_link
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = self.orientation_tolerance
        oc.absolute_y_axis_tolerance = self.orientation_tolerance
        oc.absolute_z_axis_tolerance = self.orientation_tolerance
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        return self._send_motion_plan_request(
            self._make_motion_plan_request(constraints))

    def _plan_joints(self, joint_positions: dict):
        constraints = Constraints()
        for name, value in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        return self._send_motion_plan_request(
            self._make_motion_plan_request(constraints))

    # ── 실행 ─────────────────────────────────────────────────────────────────
    def _execute(self, trajectory) -> bool:
        if not self.execute_after_plan:
            self.get_logger().info('execute_after_plan=false: RViz 계획 표시만 수행')
            return True

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        fut_gh = self.exec_client.send_goal_async(goal)
        if not self._await(fut_gh, timeout=10.0):
            self.get_logger().error('실행 goal 전송 타임아웃')
            return False
        gh = fut_gh.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('실행 goal 거부됨')
            return False

        self.get_logger().info('실행 중...')
        fut_res = gh.get_result_async()
        if not self._await(fut_res, timeout=60.0):
            self.get_logger().error('실행 타임아웃')
            return False
        res = fut_res.result()
        if res and res.result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('이동 완료')
            return True
        else:
            code = res.result.error_code.val if res else -1
            self.get_logger().error(f'이동 실패 (code={code})')
            return False

    def _run_post_grasp_sequence(self):
        self._publish_status('후속 grasp sequence 시작')
        if self.force_open_before_close:
            self._publish_status('[2/11] close 전 수동 open 위치 재고정')
            if not self._hold_manual_gripper_open():
                self.get_logger().error('close 전 gripper open 실패 — 후속 이동 중단')
                return
            time.sleep(self.gripper_settle_sec)

        if self.grasp_roll_enabled:
            self._publish_status('[3/11] grasp angle 기준 joint6 회전')
            if not self._run_grasp_roll_alignment():
                return

        if self.force_open_before_close:
            self._publish_status('[4/11] 회전 후 수동 open 위치 재고정')
            if not self._hold_manual_gripper_open():
                self.get_logger().error('회전 후 gripper open 실패 — 후속 이동 중단')
                return
            time.sleep(self.gripper_settle_sec)

        self._publish_status('[5/11] 그리퍼 닫기')
        if not self._control_gripper(open_gripper=False):
            self.get_logger().error('그리퍼 close 실패 — 후속 이동 중단')
            return
        time.sleep(self.gripper_settle_sec)

        self._publish_status(
            f'[6/11] carry pose로 들어올림 (J3 {self.carry_elbow_extra_bend_deg:.1f}deg 추가 굽힘)')
        trajectory = self._plan_joints(self._carry_joints())
        if trajectory is None or not self._execute(trajectory):
            self.get_logger().error('carry pose 이동 실패')
            return
        self._clear_display_after_execute()
        self._sync_planning_scene()
        time.sleep(self.post_grasp_settle_sec)

        self._publish_status('[7/11] place pose로 이동')
        if not self._run_place_transition():
            return

        if self.post_place_turn_enabled:
            self._publish_status('[8/11] place 후 J1 오른쪽 회전')
            if not self._run_post_place_turn():
                return

        if self.open_after_place:
            if self.pre_open_after_place_delay_sec > 0.0:
                self._publish_status(
                    f'[9/11] 그리퍼 open 전 {self.pre_open_after_place_delay_sec:.1f}s 대기')
                time.sleep(self.pre_open_after_place_delay_sec)
            self._publish_status('[9/11] 그리퍼 열기')
            if not self._hold_manual_gripper_open():
                self.get_logger().error('place 위치에서 그리퍼 open 실패')
                return
            time.sleep(self.gripper_settle_sec)

        if self.post_place_turn_enabled:
            self._publish_status('[10/11] place pose로 복귀')
            trajectory = self._plan_joints(self.place_joints)
            if trajectory is None or not self._execute(trajectory):
                self.get_logger().error('place 복귀 이동 실패')
                return
            self._clear_display_after_execute()
            self._sync_planning_scene()
            time.sleep(self.post_grasp_settle_sec)

        if self.return_ready_after_place:
            self._publish_status('[11/11] ready pose로 복귀')
            trajectory = self._plan_joints(self._READY_JOINTS)
            if trajectory is not None and self._execute(trajectory):
                self._clear_display_after_execute()
                self._sync_planning_scene()

        self._publish_status('후속 grasp sequence 완료')

    def _run_grasp_roll_alignment(self) -> bool:
        with self._grasp_roll_lock:
            roll_deg = self._latest_grasp_roll_deg
            stamp = self._latest_grasp_roll_stamp

        if roll_deg is None or stamp is None:
            self.get_logger().warn('grasp_info angle 없음 — joint6 회전 생략')
            self._publish_status('grasp angle 없음: joint6 회전 생략')
            return True

        age = time.monotonic() - stamp
        if self.grasp_roll_max_age_sec > 0.0 and age > self.grasp_roll_max_age_sec:
            self.get_logger().warn(
                f'grasp_info angle이 오래됨({age:.1f}s) — joint6 회전 생략')
            self._publish_status('grasp angle 오래됨: joint6 회전 생략')
            return True

        current = self._current_arm_positions()
        if self.grasp_roll_joint_name not in current:
            self.get_logger().error(
                f'{self.grasp_roll_joint_name} 현재 joint state 없음 — joint6 회전 중단')
            return False

        adjusted_deg = roll_deg * self.grasp_roll_sign + self.grasp_roll_offset_deg
        limited_deg = max(
            -self.grasp_roll_max_abs_deg,
            min(self.grasp_roll_max_abs_deg, adjusted_deg),
        )
        if abs(limited_deg - adjusted_deg) > 1e-3:
            self.get_logger().warn(
                f'grasp roll 제한 적용: {adjusted_deg:.1f}deg -> {limited_deg:.1f}deg')

        target_joints = dict(current)
        target_joints[self.grasp_roll_joint_name] = (
            current[self.grasp_roll_joint_name] + math.radians(limited_deg)
        )
        target_joints = self._normalize_joint_targets(target_joints)

        self._publish_status(
            f'joint6 grasp 회전: angle={roll_deg:.1f}deg, command={limited_deg:.1f}deg')
        trajectory = self._plan_joints(target_joints)
        if trajectory is None or not self._execute(trajectory):
            self.get_logger().error('grasp angle joint6 회전 실패')
            return False
        self._clear_display_after_execute()
        self._sync_planning_scene()
        time.sleep(self.post_grasp_settle_sec)
        return True

    def _run_place_transition(self) -> bool:
        if self.place_transition_mode == 'load_turn':
            turn_joints = self._carry_joints()
            if self.load_turn_joint_name not in turn_joints:
                self.get_logger().error(
                    f'load_turn_joint_name={self.load_turn_joint_name} 는 arm joint가 아님')
                return False
            turn_joints[self.load_turn_joint_name] += self.load_turn_angle

            self._publish_status(
                f'place 중간 자세: {self.load_turn_joint_name} '
                f'+{self.load_turn_angle:.3f}rad')
            trajectory = self._plan_joints(turn_joints)
            if trajectory is None or not self._execute(trajectory):
                self.get_logger().error('place 중간 자세 이동 실패')
                return False
            self._clear_display_after_execute()
            self._sync_planning_scene()
            time.sleep(self.post_grasp_settle_sec)

        trajectory = self._plan_joints(self.place_joints)
        if trajectory is None or not self._execute(trajectory):
            self.get_logger().error('place pose 이동 실패')
            return False
        self._clear_display_after_execute()
        self._sync_planning_scene()
        time.sleep(self.post_grasp_settle_sec)
        return True

    def _run_post_place_turn(self) -> bool:
        turn_joints = dict(self.place_joints)
        if self.post_place_turn_joint_name not in turn_joints:
            self.get_logger().error(
                f'post_place_turn_joint_name={self.post_place_turn_joint_name} 는 arm joint가 아님')
            return False
        turn_joints[self.post_place_turn_joint_name] += self.post_place_turn_angle

        self._publish_status(
            f'place 회전 자세: {self.post_place_turn_joint_name} '
            f'{self.post_place_turn_angle:+.3f}rad')
        trajectory = self._plan_joints(turn_joints)
        if trajectory is None or not self._execute(trajectory):
            self.get_logger().error('place 후 회전 자세 이동 실패')
            return False
        self._clear_display_after_execute()
        self._sync_planning_scene()
        time.sleep(self.post_grasp_settle_sec)
        return True

    def _current_joint_position(self, joint_name: str) -> float | None:
        with self._js_lock:
            js = self._js
        if js is None:
            return None
        for name, position in zip(js.name, js.position):
            if name == joint_name:
                return float(position)
        return None

    def _hold_manual_gripper_open(self) -> bool:
        if not self.hold_manual_gripper_open:
            return self._control_gripper(open_gripper=True)

        if self._manual_gripper_open_position is None:
            position = self._current_joint_position(self.gripper_hold_joint_name)
            if position is None:
                self.get_logger().error(
                    f'{self.gripper_hold_joint_name} 현재 joint state 없음 — 수동 open 위치 고정 실패')
                return False
            self._manual_gripper_open_position = position
            self._publish_status(
                f'수동 gripper open 기준 저장: {self.gripper_hold_joint_name}={position:.4f}')

        return self._control_gripper_position(
            self._manual_gripper_open_position,
            'hold-open',
        )

    def _control_gripper(self, open_gripper: bool) -> bool:
        position = self.gripper_open_position if open_gripper else self.gripper_close_position
        action = 'open' if open_gripper else 'close'
        return self._control_gripper_position(position, action)

    def _control_gripper_position(self, position: float, action: str) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self.gripper_max_effort

        self.get_logger().info(f'그리퍼 {action} 명령 전송: position={position:.4f}')
        fut_gh = self.gripper_client.send_goal_async(goal)
        if not self._await(fut_gh, timeout=5.0):
            self.get_logger().error(f'그리퍼 {action} goal 전송 타임아웃')
            return False

        gh = fut_gh.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'그리퍼 {action} goal 거부됨')
            return False

        fut_res = gh.get_result_async()
        if not self._await(fut_res, timeout=10.0):
            self.get_logger().error(f'그리퍼 {action} 동작 타임아웃')
            return False

        self.get_logger().info(f'그리퍼 {action} 완료')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = VisionMove()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
