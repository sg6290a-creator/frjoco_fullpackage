#!/usr/bin/env python3
"""Main simulation launch: MoveIt/RViz, simulated mobile Nav2, and web UI."""

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
                    FindPackageShare('frjoco_bringup'),
                    'launch',
                    'internal',
                    'main_sim_impl.launch.py',
                ])
            )
        )
    ])
