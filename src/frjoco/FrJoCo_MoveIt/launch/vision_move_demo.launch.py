#!/usr/bin/env python3
"""
vision_move_demo.launch.py

RealSense D405 + YOLO + 가상 카메라 TF + MoveIt + 실 팔 + vision_move 노드 통합.

사전 조건:
  sudo modprobe gs_usb
  sudo ip link set can2 up type can bitrate 1000000
  RealSense D405 USB 연결 (실제 위치 무관, 가상으로 EE에 부착됐다고 가정)

사용법:
  ros2 launch mobile_manipulator_moveit_config vision_move_demo.launch.py
  ros2 launch mobile_manipulator_moveit_config vision_move_demo.launch.py target:=2
  ros2 launch mobile_manipulator_moveit_config vision_move_demo.launch.py rviz:=false

파이프라인:
  RealSense → YOLO 추론 → can/box/phone_target_point (frame=d405_optical_frame)
  static_TF: tool0 → d405_optical_frame   (가상 카메라 마운트)
  vision_move: PointStamped 구독 → TF 변환 → MoveGroup → JTC → CAN → 실 팔
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# target 인자 → YOLO 토픽 매핑
TARGET_MAP = {
    '1': 'can_target_point',
    '2': 'box_target_point',
    '3': 'phone_target_point',
}


def launch_setup(context, *args, **kwargs):
    can_interface = LaunchConfiguration('can_interface').perform(context)
    target        = LaunchConfiguration('target').perform(context)
    rviz          = LaunchConfiguration('rviz').perform(context)
    model_path    = LaunchConfiguration('model_path').perform(context)
    keep_orient   = LaunchConfiguration('keep_orientation').perform(context)
    show_yolo_vis = LaunchConfiguration('show_yolo_visualization').perform(context)

    target_topic = TARGET_MAP.get(target, 'can_target_point')

    # ── 1. 팔 + MoveIt (HW + ros2_control + JTC + move_group + RViz) ───────
    arm_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mobile_manipulator_moveit_config'),
                'launch',
                'frlab_arm_moveit.launch.py',
            ])
        ),
        launch_arguments={
            'can_interface': can_interface,
            'rviz':          rviz,
        }.items(),
    )

    # ── 2. RealSense D405 ─────────────────────────────────────────────────
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable':  'false',
        }.items(),
    )

    # ── 3. 가상 카메라 TF (tool0 → d405_optical_frame) ─────────────────────
    #   실제 카메라 미부착 상태에서 YOLO 좌표를 EE 기준으로 처리하기 위한 임시 TF.
    #   ROS optical convention: roll=-pi/2, yaw=-pi/2 (Z=forward, X=right, Y=down)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tool0_to_camera',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
            '--frame-id', 'tool0',
            '--child-frame-id', 'd405_optical_frame',
        ],
        output='log',
    )

    # ── 4. YOLO 노드 (PointStamped publish) ────────────────────────────────
    yolo = Node(
        package='yolo_realsense',
        executable='yolo_node',
        name='yolo_publisher',
        output='screen',
        parameters=[{
            'model_path':           model_path,
            'camera_frame':         'd405_optical_frame',
            'show_visualization':   show_yolo_vis.lower() == 'true',
        }],
    )

    # ── 5. vision_move 노드 (target topic → /target_point remap) ───────────
    vision_move = Node(
        package='mobile_manipulator_moveit_config',
        executable='vision_move.py',
        name='vision_move',
        output='screen',
        parameters=[{
            'planning_group':   'arm',
            'planning_frame':   'base_link',
            'tip_link':         'tool0',
            'keep_orientation': keep_orient.lower() == 'true',
            'input_topic':      '/' + target_topic,
        }],
    )

    return [
        arm_moveit,
        TimerAction(period=3.0,  actions=[realsense]),
        TimerAction(period=4.0,  actions=[static_tf]),
        TimerAction(period=8.0,  actions=[yolo]),
        TimerAction(period=12.0, actions=[vision_move]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target',        default_value='1',
                              description='1=can, 2=box, 3=phone'),
        DeclareLaunchArgument('can_interface', default_value='can2'),
        DeclareLaunchArgument('rviz',          default_value='true'),
        DeclareLaunchArgument('model_path',    default_value='',
                              description='YOLO 모델 경로 (.pt or .engine). 비어있으면 패키지 default 사용.'),
        DeclareLaunchArgument('keep_orientation',         default_value='true'),
        DeclareLaunchArgument('show_yolo_visualization',  default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
