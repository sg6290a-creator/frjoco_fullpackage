#!/usr/bin/env python3
"""Arm simulation launch: MoveIt/RViz with fake joint state publishing."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mobile_manipulator_moveit_config'),
                    'launch',
                    'mobile_manipulator_planning_demo.launch.py',
                ])
            ),
            launch_arguments={
                'rviz': 'true',
                'joint_gui': 'false',
                'publish_world_base_tf': 'true',
            }.items(),
        )
    ])
