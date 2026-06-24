#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('default_velocity', default_value='0.5'),
        DeclareLaunchArgument('port_front', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('port_rear', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('front_driver_id', default_value='1'),
        DeclareLaunchArgument('rear_driver_id', default_value='2'),
        DeclareLaunchArgument('mobile_baudrate', default_value='57600'),
        DeclareLaunchArgument(
            'gripper_port',
            default_value='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4ZHT-if00-port0',
        ),
        DeclareLaunchArgument('gripper_baudrate', default_value='1000000'),
        DeclareLaunchArgument('gripper_id', default_value='15'),
        DeclareLaunchArgument('gripper_mount_x', default_value='-0.020'),
        DeclareLaunchArgument('gripper_mount_y', default_value='0.000'),
        DeclareLaunchArgument('gripper_mount_z', default_value='0.000'),
        DeclareLaunchArgument('gripper_mount_roll', default_value='0.000'),
        DeclareLaunchArgument('gripper_mount_pitch', default_value='0.000'),
        DeclareLaunchArgument('gripper_mount_yaw', default_value='0.000'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('require_confirmation', default_value='false'),
        DeclareLaunchArgument('load_turn_angle', default_value='1.5708'),
    ]

    full_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mobile_manipulator_moveit_config'),
                'launch',
                'mobile_manipulator_full_moveit.launch.py',
            ])
        ),
        launch_arguments={
            'can_interface': LaunchConfiguration('can_interface'),
            'default_velocity': LaunchConfiguration('default_velocity'),
            'port_front': LaunchConfiguration('port_front'),
            'port_rear': LaunchConfiguration('port_rear'),
            'front_driver_id': LaunchConfiguration('front_driver_id'),
            'rear_driver_id': LaunchConfiguration('rear_driver_id'),
            'mobile_baudrate': LaunchConfiguration('mobile_baudrate'),
            'gripper_port': LaunchConfiguration('gripper_port'),
            'gripper_baudrate': LaunchConfiguration('gripper_baudrate'),
            'gripper_id': LaunchConfiguration('gripper_id'),
            'gripper_mount_x': LaunchConfiguration('gripper_mount_x'),
            'gripper_mount_y': LaunchConfiguration('gripper_mount_y'),
            'gripper_mount_z': LaunchConfiguration('gripper_mount_z'),
            'gripper_mount_roll': LaunchConfiguration('gripper_mount_roll'),
            'gripper_mount_pitch': LaunchConfiguration('gripper_mount_pitch'),
            'gripper_mount_yaw': LaunchConfiguration('gripper_mount_yaw'),
            'rviz': LaunchConfiguration('rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    yolo_bridge = Node(
        package='mobile_manipulator_moveit_config',
        executable='yolo_grasp_pose_bridge.py',
        name='yolo_grasp_pose_bridge',
        output='screen',
        parameters=[{
            'output_topic': '/target_pose',
            'execute_service': '/execute_pick_place',
            'camera_frame': 'd405_optical_frame',
            'target_object': 'latest',
            'require_roll': True,
            'roll_axis': 'z',
        }],
    )

    vision_move = Node(
        package='mobile_manipulator_moveit_config',
        executable='vision_move.py',
        name='vision_move',
        output='screen',
        parameters=[{
            'planning_group': 'arm',
            'planning_frame': 'base_link',
            'tip_link': 'end_effector_link',
            'input_topic': '/target_pose',
            'use_input_orientation': True,
            'require_confirmation': ParameterValue(
                LaunchConfiguration('require_confirmation'),
                value_type=bool,
            ),
            'home_on_start': False,
            'run_grasp_sequence': True,
            'status_topic': '/pick_place_status',
            'open_before_grasp': True,
            'gripper_open_position': 0.029,
            'gripper_close_position': 0.020,
            'gripper_max_effort': 50.0,
            'load_turn_joint_name': 'shoulder_pan_joint',
            'load_turn_angle': ParameterValue(
                LaunchConfiguration('load_turn_angle'),
                value_type=float,
            ),
            'open_after_place': True,
            'return_ready_after_place': True,
            'gripper_settle_sec': 1.0,
            'post_grasp_settle_sec': 0.5,
        }],
    )

    return LaunchDescription(args + [
        full_moveit,
        TimerAction(period=10.0, actions=[yolo_bridge]),
        TimerAction(period=12.0, actions=[vision_move]),
    ])
