#!/usr/bin/env python3
"""Sensor hardware launch: RealSense camera stack and Livox MID360."""

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
                    'sensors_hardware_impl.launch.py',
                ])
            )
        )
    ])
