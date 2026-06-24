#!/usr/bin/env python3
"""
frlab_arm_moveit.launch.py
UR5e 실 하드웨어 + MoveIt2 통합 런치

사전 조건:
  sudo modprobe gs_usb
  sudo ip link set can2 up type can bitrate 1000000

사용법:
  ros2 launch mobile_manipulator_moveit_config frlab_arm_moveit.launch.py
  ros2 launch mobile_manipulator_moveit_config frlab_arm_moveit.launch.py can_interface:=can2
  ros2 launch mobile_manipulator_moveit_config frlab_arm_moveit.launch.py rviz:=false

토픽 흐름:
  MoveIt RViz  →  /joint_trajectory_controller/follow_joint_trajectory (action)
               →  joint_trajectory_controller  →  FrlabManipulatorHardware
               →  SocketCAN (can2)  →  실 하드웨어

  /joint_states  ←  joint_state_broadcaster  ←  FrlabManipulatorHardware
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    pkg = get_package_share_directory(package_name)
    with open(os.path.join(pkg, file_path), 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('can_interface', default_value='can2',
                              description='SocketCAN 인터페이스 이름'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='MoveIt RViz 실행 여부'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('publish_camera_tf', default_value='true',
                              description='tool0 기준 D405 optical frame static TF 발행 여부'),
        DeclareLaunchArgument('camera_parent_frame', default_value='tool0',
                              description='D405 optical frame의 부모 frame'),
        DeclareLaunchArgument('camera_frame', default_value='d405_optical_frame',
                              description='D405 optical frame 이름'),
        DeclareLaunchArgument('camera_x', default_value='0.0',
                              description='camera_parent_frame 기준 D405 optical frame X 오프셋 [m]'),
        DeclareLaunchArgument('camera_y', default_value='0.0',
                              description='camera_parent_frame 기준 D405 optical frame Y 오프셋 [m]'),
        DeclareLaunchArgument('camera_z', default_value='0.0',
                              description='camera_parent_frame 기준 D405 optical frame Z 오프셋 [m]'),
        DeclareLaunchArgument('camera_roll', default_value='0.0',
                              description='camera_parent_frame 기준 D405 optical frame roll [rad]'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0',
                              description='camera_parent_frame 기준 D405 optical frame pitch [rad]'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0',
                              description='camera_parent_frame 기준 D405 optical frame yaw [rad]'),
    ]

    can_interface = LaunchConfiguration('can_interface')
    rviz_arg      = LaunchConfiguration('rviz')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    log_level     = LaunchConfiguration('log_level')
    publish_camera_tf = LaunchConfiguration('publish_camera_tf')
    camera_parent_frame = LaunchConfiguration('camera_parent_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    # ── 1. 하드웨어: ros2_control_node + joint_trajectory_controller ────────
    #   frlab_arm_real.launch.py 가 RSP, controller_manager, JTC spawner 담당
    #   rviz:=false → basic_display.rviz 억제 (MoveIt RViz 로 대체)
    arm_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('frjoco_bringup'),
                'launch',
                'frlab_arm_real.launch.py',
            ])
        ),
        launch_arguments={
            'can_interface': can_interface,
            'controller':    'jtc',
            'rviz':          'false',
        }.items(),
    )

    # ── 2. MoveIt 파라미터 로드 ──────────────────────────────────────────────
    MOVEIT_PKG = 'mobile_manipulator_moveit_config'

    desc_pkg  = get_package_share_directory('manipulator_description')
    urdf_path = os.path.join(desc_pkg, 'URDF', 'manipulator_urdf', 'ur5e_arm_only.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = {'robot_description': f.read()}

    moveit_pkg_path = get_package_share_directory(MOVEIT_PKG)
    with open(os.path.join(moveit_pkg_path, 'config', 'ur5e_arm_only.srdf'), 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml      = load_yaml(MOVEIT_PKG, 'config/kinematics.yaml')
    joint_limits_yaml    = load_yaml(MOVEIT_PKG, 'config/joint_limits.yaml')
    ompl_yaml            = load_yaml(MOVEIT_PKG, 'config/ompl_planning.yaml')
    controllers_yaml     = load_yaml(MOVEIT_PKG, 'config/moveit_controllers.yaml')

    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    robot_description_planning = {
        'robot_description_planning': joint_limits_yaml
    }

    # MoveIt 2 planning pipeline 설정을 명시해 CHOMP 자동 선택을 막고 OMPL을 사용한다.
    ompl_pipeline = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': ompl_yaml,
    }

    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_yaml.get(
            'moveit_simple_controller_manager', {}),
        'moveit_controller_manager': controllers_yaml.get(
            'moveit_controller_manager',
            'moveit_simple_controller_manager/MoveItSimpleControllerManager'),
    }

    # 실 하드웨어용 설정
    #   moveit_manage_controllers: False → ros2_control 이 컨트롤러 관리
    #   tolerances 는 시뮬보다 넉넉하게
    trajectory_execution = {
        'moveit_manage_controllers':                                False,
        'trajectory_execution.allowed_execution_duration_scaling': 4.0,
        'trajectory_execution.allowed_goal_duration_margin':       2.0,
        'trajectory_execution.allowed_start_tolerance':            0.1,
    }

    planning_scene_monitor = {
        'publish_planning_scene':     True,
        'publish_geometry_updates':   True,
        'publish_state_updates':      True,
        'publish_transforms_updates': True,
    }

    # ── 3. move_group 노드 ───────────────────────────────────────────────────
    #   JTC 가 뜨기까지 여유를 두고 5 초 뒤에 시작
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_pipeline,
            trajectory_execution,
            planning_scene_monitor,
            moveit_controllers,
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── 4. RViz (MoveIt 전용 config) ─────────────────────────────────────────
    rviz_config = os.path.join(moveit_pkg_path, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
    )

    world_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        output='screen',
        arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
    )

    # Active URDF is arm-only and ends at tool0, so publish the camera mount
    # transform separately. Until the D405 mount is measured, defaults are
    # identity: tool0 and d405_optical_frame are treated as the same frame.
    tool0_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tool0_to_d405_optical_frame',
        output='screen',
        condition=IfCondition(publish_camera_tf),
        arguments=[
            '--x', camera_x,
            '--y', camera_y,
            '--z', camera_z,
            '--roll', camera_roll,
            '--pitch', camera_pitch,
            '--yaw', camera_yaw,
            '--frame-id', camera_parent_frame,
            '--child-frame-id', camera_frame,
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            world_to_base_link,
            tool0_to_camera,
            arm_real,
            TimerAction(period=5.0, actions=[move_group]),
            TimerAction(period=8.0, actions=[rviz_node]),
        ]
    )
