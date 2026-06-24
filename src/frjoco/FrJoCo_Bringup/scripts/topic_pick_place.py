#!/usr/bin/env python3
"""
============================================================================
Pick and Place Controller - UI Triggered YOLO Target Receiver
============================================================================

YOLO/grasp 노드에서 받은 PointStamped 좌표를 저장하고, 웹 UI의 실행 요청을
받아 기존 pick and place 시퀀스를 호출한다.

주의:
- 이 노드는 내부 TF를 수행하지 않는다.
- PointStamped.header.frame_id는 로그/상태용으로 보존한다.
- 현재 motion sequence는 legacy 4-DOF 상대 조인트 이동 로직이므로,
  UR5e MoveIt 기반 실기 파지는 vision_move.py 또는 별도 MoveIt bridge로
  대체/검증해야 한다.

[동작 순서]
1. 객체별 PointStamped 토픽으로 YOLO/grasp 좌표 수신
2. frame_id와 x/y/z 저장
3. 웹 UI가 /execute_pick_place 호출
4. 저장된 좌표로 기존 pick and place sequence 호출

[사용법]
ros2 launch frjoco_bringup moveit.launch.py
ros2 run frjoco_bringup topic_pick_place.py

[토픽]
- 입력: /hammer_target_point, /pliers_target_point, /screwdriver_target_point
- 트리거: /execute_pick_place (std_srvs/Trigger)
- 출력: /pick_place_status (std_msgs/String) - 상태 메시지

============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.duration import Duration as RCLPYDuration

from geometry_msgs.msg import Point, PointStamped, Pose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState

from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint,
    BoundingVolume, MoveItErrorCodes, RobotTrajectory,
    JointConstraint, PlanningScene, PlanningSceneComponents,
    CollisionObject, AttachedCollisionObject
)
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, ApplyPlanningScene
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from control_msgs.action import GripperCommand, FollowJointTrajectory
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import math
import time
import copy
import numpy as np


class TopicPickAndPlace(Node):
    def __init__(self):
        super().__init__('topic_pick_place')

        # ================================================================
        # Parameters
        # ================================================================
        self.declare_parameter('approach_height', 0.15)      # 접근 거리 (X축 뒤로 15cm)
        self.declare_parameter('retreat_height', 0.20)       # 후퇴 거리 (X축 뒤로 20cm)
        self.declare_parameter('pre_approach_distance', 0.05) # 잡기 전 뒤로 물러나는 거리 (5cm)
        self.declare_parameter('gripper_open_position', 0.030)
        self.declare_parameter('gripper_close_position', 0.000)
        self.declare_parameter('gripper_max_effort', 50.0)
        self.declare_parameter('camera_frame', 'd405_optical_frame')  # 올바른 프레임
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('end_effector_link', 'end_effector_link')  # MoveIt IK용 링크 (SRDF tip link)
        
        # Place 위치 (base_link 기준 고정 위치)
        self.declare_parameter('place_x', 0.0)
        self.declare_parameter('place_y', 0.25)
        self.declare_parameter('place_z', 0.15)

        self.approach_height = self.get_parameter('approach_height').value
        self.retreat_height = self.get_parameter('retreat_height').value
        self.pre_approach_distance = self.get_parameter('pre_approach_distance').value
        self.gripper_open_position = self.get_parameter('gripper_open_position').value
        self.gripper_close_position = self.get_parameter('gripper_close_position').value
        self.gripper_max_effort = self.get_parameter('gripper_max_effort').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.place_x = self.get_parameter('place_x').value
        self.place_y = self.get_parameter('place_y').value
        self.place_z = self.get_parameter('place_z').value

        self.callback_group = ReentrantCallbackGroup()
        self._busy = False

        # 중복 타겟 방지를 위한 변수들
        self.last_target_time = None
        self.target_cooldown = 2.0  # 2초 쿨다운
        self.last_target_position = None
        self.position_threshold = 0.05  # 5cm 이내 동일 위치 무시

        # 감지된 목표 좌표 저장 (버튼 클릭 시 사용)
        self.detected_target = None
        self.detected_target_frame = None
        self.auto_execute = False  # 자동 실행 비활성화

        # ================================================================
        # Joint states
        # ================================================================
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Arm joint names (from SRDF)
        self.arm_joints = [
            'link2_to_link1',
            'link3_to_link2',
            'link4_to_link3',
            'gripper_to_link4'
        ]

        # Ready pose (from SRDF - singularity-free)
        self.ready_positions = [0.0, -0.69813, -2.35619, 0.05236]

        # Place pose (물체를 놓는 위치) - 현재 위치 기반
        self.place_positions = [3.12413936106985, -0.20943951023931953, -2.548180707911721, 0.03490658503988659]

        # ================================================================
        # MoveIt clients
        # ================================================================
        self.plan_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path',
            callback_group=self.callback_group
        )

        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.callback_group
        )

        self.execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
            callback_group=self.callback_group
        )

        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd',
            callback_group=self.callback_group
        )

        self.arm_trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/arm/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Octomap clear service (타겟 주변 장애물 제거용)
        self.clear_octomap_client = self.create_client(
            Trigger, '/clear_octomap',
            callback_group=self.callback_group
        )
        
        # Planning Scene service (타겟을 collision-free zone으로 추가)
        self.apply_planning_scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene',
            callback_group=self.callback_group
        )

        # ================================================================
        # Wait for services
        # ================================================================
        self.get_logger().info('Waiting for MoveIt services...')
        self.plan_client.wait_for_service(timeout_sec=30.0)
        self.move_group_client.wait_for_server(timeout_sec=30.0)
        self.execute_client.wait_for_server(timeout_sec=30.0)
        # 그리퍼 액션 서버 대기 (선택적)
        self.get_logger().info('Waiting for gripper action server (optional)...')
        if self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Gripper action server ready!')
        else:
            self.get_logger().warn('Gripper action server not available - will use simulation mode')
        self.arm_trajectory_client.wait_for_server(timeout_sec=30.0)
        self.get_logger().info('All services ready!')

        # ================================================================
        # Subscribers
        # ================================================================
        # PointStamped (with frame_id) - YOLO grasp 좌표 (3개 객체별)
        self.hammer_target_sub = self.create_subscription(
            PointStamped, '/hammer_target_point', self.target_stamped_callback, 10,
            callback_group=self.callback_group
        )

        self.pliers_target_sub = self.create_subscription(
            PointStamped, '/pliers_target_point', self.target_stamped_callback, 10,
            callback_group=self.callback_group
        )

        self.screwdriver_target_sub = self.create_subscription(
            PointStamped, '/screwdriver_target_point', self.target_stamped_callback, 10,
            callback_group=self.callback_group
        )

        # Point (without frame_id) - base_link 기준으로 가정 (legacy support)
        self.target_point_sub = self.create_subscription(
            Point, '/target_position', self.target_point_callback, 10,
            callback_group=self.callback_group
        )

        # Status publisher
        self.status_pub = self.create_publisher(String, '/pick_place_status', 10)

        # Execute service (웹 UI 버튼용)
        self.execute_srv = self.create_service(
            Trigger, '/execute_pick_place', self.execute_pick_place_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('='*60)
        self.get_logger().info('Pick and Place Controller Ready!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Camera frame: {self.camera_frame}')
        self.get_logger().info(f'Base frame: {self.base_frame}')
        self.get_logger().info(f'Place position (base_link): ({self.place_x}, {self.place_y}, {self.place_z})')
        self.get_logger().info('')
        self.get_logger().info('Input topics (frame is preserved; no TF is performed in this node):')
        self.get_logger().info('  /hammer_target_point (PointStamped) - HAMMER object')
        self.get_logger().info('  /pliers_target_point (PointStamped) - PLIERS object')
        self.get_logger().info('  /screwdriver_target_point (PointStamped) - SCREWDRIVER object')
        self.get_logger().info('  /target_position (Point) - legacy support')
        self.get_logger().info('='*60)

    # ================================================================
    # Callbacks
    # ================================================================
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg

    def publish_status(self, message: str):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(f'[Status] {message}')

    def execute_pick_place_callback(self, request, response):
        """웹 UI에서 Pick & Place 버튼 클릭 시 호출"""
        if self._busy:
            response.success = False
            response.message = '이미 실행 중입니다'
            return response

        if self.detected_target is None:
            response.success = False
            response.message = '감지된 목표가 없습니다. 먼저 물체를 감지하세요.'
            return response

        if self.detected_target is None or len(self.detected_target) != 3:
            response.success = False
            response.message = '유효하지 않은 목표 좌표입니다.'
            return response

        self.get_logger().info(f'웹 UI에서 Pick & Place 실행 요청 (target_frame={self.detected_target_frame})')
        self._busy = True
        try:
            self.pick_and_place_sequence_relative(
                self.detected_target[0],
                self.detected_target[1],
                self.detected_target[2]
            )
            response.success = True
            response.message = 'Pick & Place 완료'
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
        finally:
            self._busy = False

        return response

    def target_stamped_callback(self, msg: PointStamped):
        """PointStamped 메시지 수신 - 내부 TF 없이 좌표와 frame_id를 그대로 저장."""
        # 중복 타겟 방지 (쿨다운 체크)
        current_time = self.get_clock().now()
        if self.last_target_time is not None:
            time_diff = (current_time - self.last_target_time).nanoseconds / 1e9
            if time_diff < self.target_cooldown:
                return  # 조용히 무시

        # 동일 위치 체크
        current_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        if self.last_target_position is not None:
            distance = np.linalg.norm(current_pos - self.last_target_position)
            if distance < self.position_threshold:
                return  # 조용히 무시

        self.get_logger().info(
            f'새로운 목표 좌표 수신: frame={msg.header.frame_id} '
            f'pos=({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})'
        )

        target_position = np.array([msg.point.x, msg.point.y, msg.point.z])

        # 좌표 저장 (버튼 클릭 시 사용). frame_id는 MoveIt/상위 노드에서 해석한다.
        self.detected_target = target_position
        self.detected_target_frame = msg.header.frame_id or self.camera_frame
        self.last_target_time = current_time
        self.last_target_position = current_pos
        self.publish_status(
            f'목표 감지({self.detected_target_frame}): '
            f'({target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f})'
        )

        # 자동 실행 모드일 경우에만 즉시 실행
        if self.auto_execute and not self._busy:
            self.get_logger().info('자동 실행 모드: Pick & Place 시작')
            self._busy = True
            try:
                self.pick_and_place_sequence_relative(target_position[0], target_position[1], target_position[2])
            except Exception as e:
                self.get_logger().error(f'Error: {str(e)}')
                self.publish_status(f'ERROR: {str(e)}')
            finally:
                self._busy = False

    def target_point_callback(self, msg: Point):
        """Point 메시지 수신 - legacy 입력. camera_frame으로 태그만 붙이고 TF는 하지 않는다."""
        if self._busy:
            self.get_logger().warn('Busy, ignoring new target')
            return

        self.get_logger().info(f'Received target ({self.camera_frame}): ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})')
        self.detected_target = np.array([msg.x, msg.y, msg.z])
        self.detected_target_frame = self.camera_frame

        self._busy = True
        try:
            self.pick_and_place_sequence_relative(msg.x, msg.y, msg.z)
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            self.publish_status(f'ERROR: {str(e)}')
        finally:
            self._busy = False

    # ================================================================
    # Motion Functions
    # ================================================================
    def move_to_pose(self, x: float, y: float, z: float, description: str = "", 
                     disable_collision_avoidance: bool = False) -> bool:
        """OMPL 플래너로 목표 위치 이동 (실패 시 바로 False 반환)
        
        Args:
            x, y, z: 목표 위치 (base_link 기준)
            description: 로그용 설명
            disable_collision_avoidance: True면 octomap 장애물 회피 비활성화 (pick 시)
        """
        self.publish_status(f'Moving to {description}: ({x:.3f}, {y:.3f}, {z:.3f})')

        # MoveIt이 사용 가능한지 확인
        if not self.move_group_client.server_is_ready():
            self.get_logger().error('MoveIt not available!')
            return False

        # OMPL RRTstar Planner 사용 (최적 경로 탐색)
        planners = ["RRTstar", "RRTConnect"]  # Optimal path first
        
        for planner in planners:
            self.get_logger().info(f'Trying OMPL planner: {planner}')
            
            goal = MoveGroup.Goal()
            goal.request.group_name = self.planning_group
            goal.request.planner_id = planner
            goal.request.num_planning_attempts = 5  # RRTstar는 더 많은 시도
            goal.request.allowed_planning_time = 10.0  # 최적화 시간
            goal.request.max_velocity_scaling_factor = 0.2
            goal.request.max_acceleration_scaling_factor = 0.2

            # RViz 방식: 순수 Pose 목표만 사용
            constraints = Constraints()
            
            # Position constraint
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = self.base_frame
            pos_constraint.link_name = self.end_effector_link
            
            bounding_volume = BoundingVolume()
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [0.005]  # 5mm tolerance
            bounding_volume.primitives.append(sphere)
            
            sphere_pose = Pose()
            sphere_pose.position.x = x
            sphere_pose.position.y = y
            sphere_pose.position.z = z
            sphere_pose.orientation.w = 1.0
            bounding_volume.primitive_poses.append(sphere_pose)
            
            pos_constraint.constraint_region = bounding_volume
            pos_constraint.weight = 1.0
            constraints.position_constraints.append(pos_constraint)
            
            goal.request.goal_constraints.append(constraints)

            # Planning options
            goal.planning_options.plan_only = False
            goal.planning_options.replan = False
            goal.planning_options.replan_attempts = 1

            # Send goal
            future = self.move_group_client.send_goal_async(goal)

            # Wait for acceptance
            timeout = 3.0  # Pilz는 빠름
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)

            if not future.done():
                self.get_logger().warn(f'TIMEOUT: Goal send for {planner}')
                continue

            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn(f'REJECTED: Goal for {planner}')
                continue

            self.get_logger().info(f'Goal accepted for {planner}, executing...')

            # Wait for result
            result_future = goal_handle.get_result_async()
            timeout = 10.0  # 실행 타임아웃
            start = time.time()
            while not result_future.done() and (time.time() - start) < timeout:
                time.sleep(0.1)

            if not result_future.done():
                self.get_logger().warn(f'TIMEOUT: Execution for {planner}')
                continue

            result = result_future.result()
            if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.publish_status(f'SUCCESS: {description} with {planner}')
                return True
            else:
                self.get_logger().warn(f'FAILED: {planner} (error: {result.result.error_code.val})')
        
        # 모든 OMPL 플래너 실패
        self.get_logger().error('All OMPL planners failed!')
        self.publish_status(f'ERROR: Cannot reach {description}')
        return False

    def move_to_joint_positions(self, positions: list, description: str = "", duration_sec: float = 3.0) -> bool:
        """조인트 위치로 직접 이동"""
        self.publish_status(f'Moving joints to {description}')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        goal.trajectory.points.append(point)

        future = self.arm_trajectory_client.send_goal_async(goal)

        timeout = 5.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)

        if not future.done():
            self.publish_status(f'TIMEOUT: Joint move goal send')
            return False

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.publish_status(f'REJECTED: Joint move goal')
            return False

        result_future = goal_handle.get_result_async()
        timeout = duration_sec + 10.0  # 여유있는 타임아웃
        start = time.time()
        while not result_future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)

        if result_future.done():
            result = result_future.result()
            if hasattr(result, 'result') and hasattr(result.result, 'error_code'):
                if result.result.error_code == 0:  # SUCCESS
                    self.publish_status(f'SUCCESS: {description}')
                    return True
                else:
                    self.publish_status(f'FAILED: {description} (error: {result.result.error_code})')
                    return False
            else:
                self.publish_status(f'SUCCESS: {description}')
                return True
        else:
            self.publish_status(f'TIMEOUT: {description}')
            return False

    def move_to_pose_joint_fallback(self, x: float, y: float, z: float, description: str = "") -> bool:
        """
        MoveIt 실패 시 Joint Trajectory로 우회
        실제 목표 좌표에 기반한 조인트 위치 계산
        """
        self.get_logger().warn(f'Using joint fallback for {description} at ({x:.3f}, {y:.3f}, {z:.3f})')
        
        # 목표 좌표에 따른 조인트 위치 계산
        # X: 앞뒤 거리에 따른 어깨 각도
        if x > 0.5:  # 50cm 이상 멀리
            joint1 = 0.0
            joint2 = -0.3  # 앞으로 더 내밀기
            joint3 = -1.8
        elif x > 0.35:  # 35-50cm 중간 거리
            joint1 = 0.0
            joint2 = -0.5
            joint3 = -2.0
        else:  # 35cm 이하 가까운 거리
            joint1 = 0.0
            joint2 = -0.8
            joint3 = -2.3
        
        # Y: 좌우 위치에 따른 회전
        if y > 0.15:  # 왼쪽
            joint1 = 0.8
        elif y < -0.15:  # 오른쪽
            joint1 = -0.8
        else:  # 중앙
            joint1 = 0.0
        
        # Z: 높이에 따른 손목 각도
        if z > 0.2:  # 높이 있는 목표
            joint4 = 0.3
        elif z < 0.05:  # 낮은 목표
            joint4 = -0.2
        else:  # 중간 높이
            joint4 = 0.05
        
        target_joints = [joint1, joint2, joint3, joint4]
        
        self.get_logger().info(f'Calculated joints for target ({x:.2f}, {y:.2f}, {z:.2f}): {target_joints}')
        
        return self.move_to_joint_positions(target_joints, f"calculated for {description}", 4.0)

    def move_to_ready(self) -> bool:
        """Ready 자세로 이동"""
        return self.move_to_joint_positions(self.ready_positions, "ready pose", 5.0)

    def move_to_place(self) -> bool:
        """Place 자세로 이동"""
        return self.move_to_joint_positions(self.place_positions, "place pose", 5.0)

    def control_gripper(self, open: bool) -> bool:
        """그리퍼 제어 - 그리퍼가 없어도 동작하도록 수정"""
        action = "Opening" if open else "Closing"
        self.publish_status(f'{action} gripper')

        # 그리퍼 액션 서버가 없으면 시뮬레이션 모드로 동작
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn('Gripper not available - simulating gripper action')
            self.publish_status(f'SIMULATED: Gripper {"opened" if open else "closed"}')
            time.sleep(1.0)  # 시뮬레이션 딜레이
            return True

        goal = GripperCommand.Goal()
        goal.command.position = self.gripper_open_position if open else self.gripper_close_position
        goal.command.max_effort = self.gripper_max_effort

        future = self.gripper_client.send_goal_async(goal)

        timeout = 5.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)

        if not future.done():
            self.publish_status('TIMEOUT: Gripper goal send - simulating')
            self.publish_status(f'SIMULATED: Gripper {"opened" if open else "closed"}')
            return True  # 타임아웃이어도 시뮬레이션으로 성공 처리

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.publish_status('REJECTED: Gripper goal - simulating')
            self.publish_status(f'SIMULATED: Gripper {"opened" if open else "closed"}')
            return True  # 거부되어도 시뮬레이션으로 성공 처리

        result_future = goal_handle.get_result_async()
        timeout = 10.0
        start = time.time()
        while not result_future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)

        if result_future.done():
            result = result_future.result()
            if hasattr(result, 'result'):
                self.publish_status(f'Gripper {"opened" if open else "closed"}')
                return True
            else:
                self.publish_status('FAILED: Gripper operation failed - simulating')
                self.publish_status(f'SIMULATED: Gripper {"opened" if open else "closed"}')
                return True  # 실패해도 시뮬레이션으로 성공 처리
        else:
            self.publish_status('TIMEOUT: Gripper operation - simulating')
            self.publish_status(f'SIMULATED: Gripper {"opened" if open else "closed"}')
            return True  # 타임아웃이어도 시뮬레이션으로 성공 처리

    def clear_octomap_around_target(self) -> bool:
        """타겟 주변 octomap 클리어 (pick 전에 호출)"""
        if not self.clear_octomap_client.service_is_ready():
            self.get_logger().warn('clear_octomap service not available')
            return False
        
        try:
            request = Trigger.Request()
            future = self.clear_octomap_client.call_async(request)
            
            timeout = 2.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            if future.done():
                result = future.result()
                if result.success:
                    self.get_logger().info('Octomap cleared successfully')
                    return True
                else:
                    self.get_logger().warn(f'Octomap clear failed: {result.message}')
            else:
                self.get_logger().warn('Octomap clear timeout')
        except Exception as e:
            self.get_logger().warn(f'Octomap clear exception: {e}')
        
        return False

    def add_target_collision_free_zone(self, x: float, y: float, z: float, radius: float = 0.15) -> bool:
        """
        타겟 주변을 collision-free zone으로 Planning Scene에 추가
        
        이 영역은 octomap의 장애물을 무시하도록 허용된 collision object로 추가됨
        
        Args:
            x, y, z: base_link 기준 타겟 위치
            radius: collision-free zone 반경 (기본 15cm)
        """
        if not self.apply_planning_scene_client.service_is_ready():
            self.get_logger().warn('apply_planning_scene service not available')
            return False
        
        try:
            # Collision object 생성 (구 형태)
            collision_obj = CollisionObject()
            collision_obj.header.frame_id = self.base_frame
            collision_obj.header.stamp = self.get_clock().now().to_msg()
            collision_obj.id = 'target_zone'
            
            # 구 형태 정의
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [radius]  # 반지름
            
            # 위치 설정
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0
            
            collision_obj.primitives.append(sphere)
            collision_obj.primitive_poses.append(pose)
            collision_obj.operation = CollisionObject.ADD
            
            # Planning Scene 생성
            planning_scene = PlanningScene()
            planning_scene.is_diff = True
            planning_scene.world.collision_objects.append(collision_obj)
            
            # Planning Scene에 추가 (octomap보다 우선순위 높음)
            request = ApplyPlanningScene.Request()
            request.scene = planning_scene
            
            future = self.apply_planning_scene_client.call_async(request)
            
            timeout = 2.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)
            
            if future.done():
                result = future.result()
                if result.success:
                    self.get_logger().info(f'Target collision-free zone added at ({x:.2f}, {y:.2f}, {z:.2f}), radius={radius}')
                    return True
                else:
                    self.get_logger().warn('Failed to add collision-free zone')
            else:
                self.get_logger().warn('Add collision-free zone timeout')
        except Exception as e:
            self.get_logger().error(f'Exception adding collision-free zone: {e}')
        
        return False

    def remove_target_collision_free_zone(self) -> bool:
        """타겟 collision-free zone 제거"""
        if not self.apply_planning_scene_client.service_is_ready():
            return False
        
        try:
            collision_obj = CollisionObject()
            collision_obj.id = 'target_zone'
            collision_obj.operation = CollisionObject.REMOVE
            
            planning_scene = PlanningScene()
            planning_scene.is_diff = True
            planning_scene.world.collision_objects.append(collision_obj)
            
            request = ApplyPlanningScene.Request()
            request.scene = planning_scene
            
            future = self.apply_planning_scene_client.call_async(request)
            
            timeout = 1.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)
            
            if future.done():
                self.get_logger().info('Target collision-free zone removed')
                return True
        except Exception as e:
            self.get_logger().warn(f'Exception removing collision-free zone: {e}')
        
        return False

    # ================================================================
    # Pick and Place Sequence (Relative to End-Effector)
    # ================================================================
    def pick_and_place_sequence_relative(self, rel_x: float, rel_y: float, rel_z: float):
        """
        end_effector_link 기준 상대 이동으로 Pick and Place 수행
        
        시퀀스:
        1. 그리퍼 열기 (준비)
        2. 상대 위치로 이동 (현재 위치 + rel_x, rel_y, rel_z)
        3. 그리퍼 닫기 (물체 잡기)
        4. Ready 자세로 이동 (물체를 들고)
        5. Place 자세로 이동
        6. 그리퍼 열기 (물체 놓기)
        7. Ready 자세로 복귀
        
        Args:
            rel_x, rel_y, rel_z: end_effector_link 기준 상대 이동량
        """
        self.publish_status('='*50)
        self.publish_status('Starting Pick & Place (Relative Movement)')
        self.publish_status(f'Relative movement (end_effector): ({rel_x:.3f}, {rel_y:.3f}, {rel_z:.3f})')
        self.publish_status('='*50)

        # 1. 그리퍼 열기 (준비)
        self.publish_status('[1/7] Opening gripper (preparing to grasp)')
        success = self.control_gripper(open=True)
        if not success:
            self.publish_status('FAILED: Could not open gripper')
            return
        self.publish_status('[1/8] DONE - Gripper opened')
        # 그리퍼 동작 완료 대기 - 딱로 대기 없이 바로 다음 단계

        # 2. 목표 위치보다 뒤로 물러난 pre-approach 위치로 이동
        pre_approach_x = rel_x - self.pre_approach_distance  # 목표보다 뒤로
        self.publish_status(f'[2/8] Moving to pre-approach position (back {self.pre_approach_distance*100:.0f}cm from target)')
        success = self.move_relative_to_gripper(pre_approach_x, rel_y, rel_z, "pre-approach position")
        if not success:
            self.publish_status('FAILED: Could not reach pre-approach position')
            self.move_to_ready()
            return
        
        # Pre-approach에서 손목(joint4)을 0도로 설정
        self.publish_status('[2/8] Setting wrist (joint4) to 0 degrees')
        if self.current_joint_state is not None:
            current_positions = []
            for joint_name in self.arm_joints:
                if joint_name in self.current_joint_state.name:
                    idx = self.current_joint_state.name.index(joint_name)
                    current_positions.append(self.current_joint_state.position[idx])
            if len(current_positions) == 4:
                # joint4만 0으로 변경
                current_positions[3] = 0.0
                self.move_to_joint_positions(current_positions, "wrist to 0 deg", duration_sec=1.5)
        
        self.publish_status('[2/8] DONE - Pre-approach position reached')
        self.get_clock().sleep_for(RCLPYDuration(seconds=2))

        # 3. 목표 좌표로 이동 (물체 잡기 직전)
        # joint4를 0도로 맞춘 후 현재 위치에서 원래 목표 좌표까지 이동
        self.publish_status(f'[3/8] Moving to target grasp position (x={rel_x:.3f}m)')
        success = self.move_relative_to_gripper(rel_x, rel_y, rel_z, "grasp position")
        if not success:
            self.publish_status('FAILED: Could not reach grasp position')
            self.move_to_ready()
            return
        self.publish_status('[3/8] DONE - Grasp position reached')
        # 이동 완료 후 안정화를 위해 딜레이 증가
        self.get_clock().sleep_for(RCLPYDuration(seconds=4))

        # 4. 그리퍼 닫기 (물체 잡기)
        self.publish_status('[4/8] Closing gripper (grasping)')
        success = self.control_gripper(open=False)
        if not success:
            self.publish_status('FAILED: Could not close gripper')
            self.move_to_ready()
            return
        self.publish_status('[4/8] DONE - Gripper closed')
        # 그리퍼 닫기 완료 후 안정화
        self.get_clock().sleep_for(RCLPYDuration(seconds=4))

        # 5. Ready 자세로 이동 (물체 들고)
        self.publish_status('[5/8] Moving to ready pose (holding object)')
        success = self.move_to_ready()
        if not success:
            self.publish_status('FAILED: Could not reach ready pose')
            return
        self.publish_status('[5/8] DONE - Ready pose reached')
        self.get_clock().sleep_for(RCLPYDuration(seconds=4))

        # 6. Place 자세로 이동
        self.publish_status('[6/8] Moving to place pose')
        success = self.move_to_place()
        if not success:
            self.publish_status('FAILED: Could not reach place pose')
            self.move_to_ready()
            return
        self.publish_status('[6/8] DONE - Place pose reached')
        self.get_clock().sleep_for(RCLPYDuration(seconds=10))

        # 7. 그리퍼 열기 (물체 놓기)
        self.publish_status('[7/8] Opening gripper (releasing object)')
        success = self.control_gripper(open=True)
        if not success:
            self.publish_status('FAILED: Could not open gripper')
            return
        self.publish_status('[7/8] DONE - Gripper opened')
        self.get_clock().sleep_for(RCLPYDuration(seconds=4))

        # 8. Ready 자세로 복귀
        self.publish_status('[8/8] Returning to ready pose')
        success = self.move_to_ready()
        if success:
            self.publish_status('[8/8] DONE - Ready pose reached')
            success = self.control_gripper(open=True)
            if not success:
                self.publish_status('WARNING: Final gripper open failed')
            self.publish_status('[8/8] Gripper ready for next task')
        
        self.publish_status('='*50)
        self.publish_status('Relative Pick & Place COMPLETED!')
        self.publish_status('='*50)

    def move_relative_to_gripper(self, rel_x: float, rel_y: float, rel_z: float, description: str = "") -> bool:
        """
        end_effector_link 기준 상대 이동 (Eye-in-hand 방식)
        
        카메라가 end_effector에 장착되어 있으므로:
        - 카메라가 본 물체 좌표 = end_effector가 이동해야 할 상대 거리
        - base_link 절대 좌표 계산 없이, 조인트 변화량으로 상대 이동
        
        Args:
            rel_x: 앞/뒤 (양수 = 앞으로) - end_effector 기준
            rel_y: 좌/우 (양수 = 왼쪽) - end_effector 기준  
            rel_z: 위/아래 (양수 = 위로) - end_effector 기준
        """
        self.publish_status(f'Relative movement (end_effector): ({rel_x:.3f}, {rel_y:.3f}, {rel_z:.3f})')

        # 안전 체크: 상대 이동 거리
        relative_distance = math.sqrt(rel_x**2 + rel_y**2 + rel_z**2)
        MAX_RELATIVE_MOVE = 0.50  # 최대 상대 이동 거리 50cm
        
        self.publish_status(f'Relative move distance: {relative_distance:.3f}m')
        
        if relative_distance > MAX_RELATIVE_MOVE:
            self.publish_status(f'ERROR: Relative move too far ({relative_distance:.3f}m > {MAX_RELATIVE_MOVE}m)')
            return False

        # 현재 조인트 위치 가져오기
        if self.current_joint_state is None:
            self.publish_status('ERROR: No joint state available')
            return False

        current_positions = []
        for joint_name in self.arm_joints:
            if joint_name in self.current_joint_state.name:
                idx = self.current_joint_state.name.index(joint_name)
                current_positions.append(self.current_joint_state.position[idx])
            else:
                self.publish_status(f'ERROR: Joint {joint_name} not found')
                return False

        self.get_logger().info(f'Current joint positions: {[f"{p:.3f}" for p in current_positions]}')

        # 조인트 변화량 계산 (Jacobian 근사)
        # end_effector 기준 상대 이동량을 조인트 변화량으로 변환
        # 
        # 4DOF 암 기구학:
        # - joint1 (link2_to_link1): base 회전 → Y축 이동에 영향
        # - joint2 (link3_to_link2): 어깨 → X, Z축 이동에 영향  
        # - joint3 (link4_to_link3): 팔꿈치 → X, Z축 이동에 영향
        # - joint4 (gripper_to_link4): 손목 → 주로 자세 조정
        
        new_positions = current_positions.copy()
        
        # 링크 길이 (URDF 기준 대략값)
        L2 = 0.13   # link2 길이
        L3 = 0.124  # link3 길이
        L4 = 0.126  # link4 + gripper
        
        # 현재 조인트 각도
        q1, q2, q3, q4 = current_positions
        
        # ================================================================
        # 조인트 방향 (URDF 기준) - 실측으로 확인됨:
        # - joint1 (Z축): +방향 = CW (위에서 보면 오른쪽으로)
        # - joint2 (Y축): +방향 = 팔이 앞으로 숙여짐
        # - joint3 (Y축): +방향 = 팔꿈치가 접힘
        # - joint4 (Y축): +방향 = 손목 아래로
        # ================================================================
        
        # X축 이동 (end_effector 기준 앞으로 = 그리퍼가 가리키는 방향)
        # Ready pose에서 팔이 앞/아래를 향함
        # 앞으로 더 가려면: joint2 증가 (어깨 숙이기), joint3 증가 (팔꿈치 펴기)
        if abs(rel_x) > 0.01:
            dq3 = rel_x / (L3 + L4)
            dq2 = rel_x / (L2 + L3)
            new_positions[2] += dq3
            new_positions[1] += dq2
            self.get_logger().info(f'X move: dq2={dq2:.3f}, dq3={dq3:.3f}')
        
        # Y축 이동 (end_effector 기준 Y+ = 왼쪽)
        # 부호 반전: 30도 방향으로 가야하는데 330도로 갔음
        if abs(rel_y) > 0.01:
            arm_length = L2 + L3 + L4
            dq1 = rel_y / arm_length
            new_positions[0] += dq1
            self.get_logger().info(f'Y move: dq1={dq1:.3f}')
        
        # Z축 이동 (위/아래) - 부호 반전
        # rel_z > 0 = 위로 가야함 → joint2 감소 (팔 들기), joint4 감소
        if abs(rel_z) > 0.01:
            dq2_z = -rel_z / (L2 + L3)
            dq4 = -rel_z / (L3 + L4)
            new_positions[1] += dq2_z
            new_positions[3] += dq4
            self.get_logger().info(f'Z move: dq2={dq2_z:.3f}, dq4={dq4:.3f}')

        # 조인트 제한 확인
        joint_limits = [
            (-3.14, 3.14),   # joint1: base 회전
            (-1.57, 1.57),   # joint2: 어깨
            (-3.14, 0.5),    # joint3: 팔꿈치
            (-1.57, 1.57)    # joint4: 손목
        ]
        
        for i, (pos, (min_l, max_l)) in enumerate(zip(new_positions, joint_limits)):
            if pos < min_l:
                new_positions[i] = min_l
                self.get_logger().warn(f'Joint {i} clamped to min: {min_l}')
            elif pos > max_l:
                new_positions[i] = max_l
                self.get_logger().warn(f'Joint {i} clamped to max: {max_l}')

        self.get_logger().info(f'Target joint positions: {[f"{p:.3f}" for p in new_positions]}')
        self.publish_status(f'✓ Moving joints to reach target...')

        # 조인트 위치로 이동
        return self.move_to_joint_positions(new_positions, description, duration_sec=4.0)

    # ================================================================
    # Pick and Place Sequence (Original - Absolute Coordinates)
    # ================================================================

def main(args=None):
    rclpy.init(args=args)

    node = TopicPickAndPlace()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
