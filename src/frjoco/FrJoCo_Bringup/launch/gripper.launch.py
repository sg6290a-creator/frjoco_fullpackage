#!/usr/bin/env python3
"""Standalone U2D2/Dynamixel gripper bringup for range tuning."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gripper_port = LaunchConfiguration('gripper_port')
    gripper_baudrate = LaunchConfiguration('gripper_baudrate')
    gripper_id = LaunchConfiguration('gripper_id')
    gripper_ping_on_activate = LaunchConfiguration('gripper_ping_on_activate')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('frjoco_bringup'),
            'config',
            'gripper_only.urdf.xacro',
        ]),
        ' ',
        'gripper_port:=', gripper_port, ' ',
        'gripper_baudrate:=', gripper_baudrate, ' ',
        'gripper_id:=', gripper_id, ' ',
        'gripper_ping_on_activate:=', gripper_ping_on_activate,
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'use_sim_time': use_sim_time,
    }

    controllers_file = PathJoinSubstitution([
        FindPackageShare('frjoco_bringup'),
        'config',
        'gripper_only_controllers.yaml',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controllers_file],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_file,
        ],
    )

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_file,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gripper_port',
            default_value='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4ZHT-if00-port0',
        ),
        DeclareLaunchArgument('gripper_baudrate', default_value='1000000'),
        DeclareLaunchArgument('gripper_id', default_value='15'),
        DeclareLaunchArgument('gripper_ping_on_activate', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        robot_state_publisher,
        controller_manager,
        TimerAction(period=2.0, actions=[joint_state_broadcaster]),
        TimerAction(period=3.0, actions=[gripper_controller]),
    ])
