#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as f:
        return yaml.safe_load(f)


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as f:
        return f.read()


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('arm_mount_x', default_value='0.200'),
        DeclareLaunchArgument('arm_mount_y', default_value='0.000'),
        DeclareLaunchArgument('arm_mount_z', default_value='0.235'),
        DeclareLaunchArgument('arm_mount_roll', default_value='0.000'),
        DeclareLaunchArgument('arm_mount_pitch', default_value='0.000'),
        DeclareLaunchArgument('arm_mount_yaw', default_value='3.1416'),
        DeclareLaunchArgument('gripper_mount_x', default_value='-0.020'),
        DeclareLaunchArgument('gripper_mount_y', default_value='0.000'),
        DeclareLaunchArgument('gripper_mount_z', default_value='0.000'),
        DeclareLaunchArgument('gripper_mount_roll', default_value='0.000'),
        DeclareLaunchArgument('gripper_mount_pitch', default_value='0.000'),
        DeclareLaunchArgument('gripper_mount_yaw', default_value='0.000'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('joint_gui', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('publish_world_base_tf', default_value='true'),
    ]

    moveit_pkg = 'mobile_manipulator_moveit_config'
    description_pkg = 'manipulator_description'

    urdf_path = PathJoinSubstitution([
        FindPackageShare(description_pkg),
        'URDF',
        'mobile_manipulator_urdf',
        'mobile_manipulator_6dof_gripper.urdf.xacro',
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_path, ' ',
        'arm_mount_x:=', LaunchConfiguration('arm_mount_x'), ' ',
        'arm_mount_y:=', LaunchConfiguration('arm_mount_y'), ' ',
        'arm_mount_z:=', LaunchConfiguration('arm_mount_z'), ' ',
        'arm_mount_roll:=', LaunchConfiguration('arm_mount_roll'), ' ',
        'arm_mount_pitch:=', LaunchConfiguration('arm_mount_pitch'), ' ',
        'arm_mount_yaw:=', LaunchConfiguration('arm_mount_yaw'), ' ',
        'gripper_mount_x:=', LaunchConfiguration('gripper_mount_x'), ' ',
        'gripper_mount_y:=', LaunchConfiguration('gripper_mount_y'), ' ',
        'gripper_mount_z:=', LaunchConfiguration('gripper_mount_z'), ' ',
        'gripper_mount_roll:=', LaunchConfiguration('gripper_mount_roll'), ' ',
        'gripper_mount_pitch:=', LaunchConfiguration('gripper_mount_pitch'), ' ',
        'gripper_mount_yaw:=', LaunchConfiguration('gripper_mount_yaw'),
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }

    robot_description_semantic = {
        'robot_description_semantic': load_file(moveit_pkg, 'config/mobile_manipulator_full.srdf'),
    }

    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml(moveit_pkg, 'config/kinematics.yaml'),
    }

    robot_description_planning = {
        'robot_description_planning': load_yaml(moveit_pkg, 'config/joint_limits.yaml'),
    }

    ompl_pipeline = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': load_yaml(moveit_pkg, 'config/ompl_planning.yaml'),
    }

    controllers_yaml = load_yaml(moveit_pkg, 'config/mobile_manipulator_full_moveit_controllers.yaml')
    moveit_controllers = {
        'moveit_controller_manager': controllers_yaml.get(
            'moveit_controller_manager',
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        ),
        'moveit_simple_controller_manager': controllers_yaml.get(
            'moveit_simple_controller_manager',
            {},
        ),
    }

    trajectory_execution = {
        'allow_trajectory_execution': True,
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 4.0,
        'trajectory_execution.allowed_goal_duration_margin': 2.0,
        'trajectory_execution.allowed_start_tolerance': 0.1,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    joint_state_defaults = {
        'zeros.shoulder_pan_joint': 0.0,
        'zeros.shoulder_lift_joint': 0.0,
        'zeros.elbow_joint': 0.0,
        'zeros.wrist_1_joint': 0.0,
        'zeros.wrist_2_joint': 0.0,
        'zeros.wrist_3_joint': 0.0,
        'zeros.gripper_left_joint': 0.030,
        'rate': 20,
    }

    robot_description_publisher = Node(
        package=moveit_pkg,
        executable='publish_robot_description.py',
        output='screen',
        parameters=[
            robot_description,
            joint_state_defaults,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    demo_joint_state_publisher = Node(
        package=moveit_pkg,
        executable='demo_joint_state_publisher.py',
        name='demo_joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('joint_gui')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('joint_gui')),
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    world_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_footprint',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'base_footprint',
        ],
        condition=IfCondition(LaunchConfiguration('publish_world_base_tf')),
    )

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
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    moveit_pkg_path = get_package_share_directory(moveit_pkg)
    rviz_config = os.path.join(moveit_pkg_path, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            world_to_base,
            robot_state_publisher,
            robot_description_publisher,
            demo_joint_state_publisher,
            joint_state_publisher_gui,
            TimerAction(period=2.0, actions=[move_group]),
            TimerAction(period=4.0, actions=[rviz_node]),
        ]
    )
