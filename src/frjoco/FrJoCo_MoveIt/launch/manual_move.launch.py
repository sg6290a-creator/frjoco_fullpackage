#!/usr/bin/env python3
"""
manual_move.launch.py
팔 하드웨어 + MoveIt + RViz + vision_move 노드.
터미널에서 terminal_target.py 로 좌표 입력 → 팔 이동.

사전 조건:
  sudo modprobe gs_usb
  sudo ip link set can2 up type can bitrate 1000000

사용법:
  ros2 launch mobile_manipulator_moveit_config manual_move.launch.py
  # 별도 터미널에서:
  ros2 run mobile_manipulator_moveit_config terminal_target.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can2'),
        DeclareLaunchArgument('rviz',          default_value='true'),

        # ── 1. 팔 + MoveIt + RViz ────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mobile_manipulator_moveit_config'),
                    'launch', 'frlab_arm_moveit.launch.py',
                ])
            ),
            launch_arguments={
                'can_interface': LaunchConfiguration('can_interface'),
                'rviz':          LaunchConfiguration('rviz'),
            }.items(),
        ),

        # ── 2. vision_move (MoveGroup action 서버가 뜬 뒤 시작) ───────────────
        TimerAction(period=15.0, actions=[
            Node(
                package='mobile_manipulator_moveit_config',
                executable='vision_move.py',
                name='vision_move',
                output='screen',
                parameters=[{
                    'planning_group':   'arm',
                    'planning_frame':   'base_link',
                    'tip_link':         'tool0',
                    'keep_orientation': True,
                    'input_topic':      '/target_point',
                }],
            ),
        ]),
    ])
