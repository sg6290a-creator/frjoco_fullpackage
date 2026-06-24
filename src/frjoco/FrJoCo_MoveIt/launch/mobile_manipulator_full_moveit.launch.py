#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
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
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('default_velocity', default_value='0.5'),
        DeclareLaunchArgument('arm_perf_log_every_n_cycles', default_value='0'),
        DeclareLaunchArgument('ros_position_offset_wrist_3_joint', default_value='0.0'),
        DeclareLaunchArgument('enable_arm_hardware', default_value='true'),
        DeclareLaunchArgument(
            'port_front',
            default_value='/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.1:1.0-port0',
        ),
        DeclareLaunchArgument(
            'port_rear',
            default_value='/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.2:1.0-port0',
        ),
        DeclareLaunchArgument('front_driver_id', default_value='1'),
        DeclareLaunchArgument('rear_driver_id', default_value='2'),
        DeclareLaunchArgument('mobile_baudrate', default_value='57600'),
        DeclareLaunchArgument('enable_mobile_hardware', default_value='false'),
        DeclareLaunchArgument(
            'gripper_port',
            default_value='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4ZHT-if00-port0',
        ),
        DeclareLaunchArgument('gripper_baudrate', default_value='1000000'),
        DeclareLaunchArgument('gripper_id', default_value='15'),
        DeclareLaunchArgument('enable_gripper_hardware', default_value='false'),
        DeclareLaunchArgument(
            'dynamixel_sdk_lib_dir',
            default_value='/home/frlab/ing_ws/src/fullpackage/install/dynamixel_sdk/lib',
        ),
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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'controllers_file',
            default_value='mobile_manipulator_full_controllers.yaml',
        ),
        DeclareLaunchArgument('publish_world_odom_tf', default_value='true'),
        DeclareLaunchArgument('publish_static_odom_base_tf', default_value='true'),
    ]

    can_interface = LaunchConfiguration('can_interface')
    default_velocity = LaunchConfiguration('default_velocity')
    arm_perf_log_every_n_cycles = LaunchConfiguration('arm_perf_log_every_n_cycles')
    ros_position_offset_wrist_3_joint = LaunchConfiguration('ros_position_offset_wrist_3_joint')
    enable_arm_hardware = LaunchConfiguration('enable_arm_hardware')
    port_front = LaunchConfiguration('port_front')
    port_rear = LaunchConfiguration('port_rear')
    front_driver_id = LaunchConfiguration('front_driver_id')
    rear_driver_id = LaunchConfiguration('rear_driver_id')
    mobile_baudrate = LaunchConfiguration('mobile_baudrate')
    enable_mobile_hardware = LaunchConfiguration('enable_mobile_hardware')
    gripper_port = LaunchConfiguration('gripper_port')
    gripper_baudrate = LaunchConfiguration('gripper_baudrate')
    gripper_id = LaunchConfiguration('gripper_id')
    enable_gripper_hardware = LaunchConfiguration('enable_gripper_hardware')
    dynamixel_sdk_lib_dir = LaunchConfiguration('dynamixel_sdk_lib_dir')
    arm_mount_x = LaunchConfiguration('arm_mount_x')
    arm_mount_y = LaunchConfiguration('arm_mount_y')
    arm_mount_z = LaunchConfiguration('arm_mount_z')
    arm_mount_roll = LaunchConfiguration('arm_mount_roll')
    arm_mount_pitch = LaunchConfiguration('arm_mount_pitch')
    arm_mount_yaw = LaunchConfiguration('arm_mount_yaw')
    gripper_mount_x = LaunchConfiguration('gripper_mount_x')
    gripper_mount_y = LaunchConfiguration('gripper_mount_y')
    gripper_mount_z = LaunchConfiguration('gripper_mount_z')
    gripper_mount_roll = LaunchConfiguration('gripper_mount_roll')
    gripper_mount_pitch = LaunchConfiguration('gripper_mount_pitch')
    gripper_mount_yaw = LaunchConfiguration('gripper_mount_yaw')
    rviz_arg = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    controllers_file = LaunchConfiguration('controllers_file')
    publish_world_odom_tf = LaunchConfiguration('publish_world_odom_tf')
    publish_static_odom_base_tf = LaunchConfiguration('publish_static_odom_base_tf')

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
        'can_interface:=', can_interface, ' ',
        'default_velocity:=', default_velocity, ' ',
        'arm_perf_log_every_n_cycles:=', arm_perf_log_every_n_cycles, ' ',
        'ros_position_offset_wrist_3_joint:=', ros_position_offset_wrist_3_joint, ' ',
        'enable_arm_hardware:=', enable_arm_hardware, ' ',
        'port_front:=', port_front, ' ',
        'port_rear:=', port_rear, ' ',
        'front_driver_id:=', front_driver_id, ' ',
        'rear_driver_id:=', rear_driver_id, ' ',
        'mobile_baudrate:=', mobile_baudrate, ' ',
        'enable_mobile_hardware:=', enable_mobile_hardware, ' ',
        'gripper_port:=', gripper_port, ' ',
        'gripper_baudrate:=', gripper_baudrate, ' ',
        'gripper_id:=', gripper_id, ' ',
        'enable_gripper_hardware:=', enable_gripper_hardware, ' ',
        'arm_mount_x:=', arm_mount_x, ' ',
        'arm_mount_y:=', arm_mount_y, ' ',
        'arm_mount_z:=', arm_mount_z, ' ',
        'arm_mount_roll:=', arm_mount_roll, ' ',
        'arm_mount_pitch:=', arm_mount_pitch, ' ',
        'arm_mount_yaw:=', arm_mount_yaw, ' ',
        'gripper_mount_x:=', gripper_mount_x, ' ',
        'gripper_mount_y:=', gripper_mount_y, ' ',
        'gripper_mount_z:=', gripper_mount_z, ' ',
        'gripper_mount_roll:=', gripper_mount_roll, ' ',
        'gripper_mount_pitch:=', gripper_mount_pitch, ' ',
        'gripper_mount_yaw:=', gripper_mount_yaw,
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic = {
        'robot_description_semantic': load_file(
            moveit_pkg,
            'config/mobile_manipulator_full.srdf',
        )
    }

    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml(moveit_pkg, 'config/kinematics.yaml')
    }

    robot_description_planning = {
        'robot_description_planning': load_yaml(moveit_pkg, 'config/joint_limits.yaml')
    }

    ompl_pipeline = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': load_yaml(moveit_pkg, 'config/ompl_planning.yaml'),
    }

    controllers_yaml = load_yaml(
        moveit_pkg,
        'config/mobile_manipulator_full_moveit_controllers.yaml',
    )
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

    full_controllers_yaml = PathJoinSubstitution([
        FindPackageShare('frjoco_bringup'),
        'config',
        controllers_file,
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        additional_env={
            'LD_LIBRARY_PATH': [
                dynamixel_sdk_lib_dir,
                ':',
                EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
            ],
        },
        parameters=[
            robot_description,
            full_controllers_yaml,
            {'use_sim_time': use_sim_time},
        ],
    )

    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'odom',
        ],
        condition=IfCondition(publish_world_odom_tf),
    )

    odom_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_static',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom',
            '--child-frame-id', 'base_footprint',
        ],
        condition=IfCondition(PythonExpression([
            "'", enable_mobile_hardware, "'.lower() != 'true' and '",
            publish_static_odom_base_tf, "'.lower() == 'true'",
        ])),
    )

    passive_joint_state_publisher = Node(
        package=moveit_pkg,
        executable='passive_joint_state_publisher.py',
        name='passive_joint_state_publisher',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", enable_arm_hardware, "'.lower() != 'true' or '",
            enable_mobile_hardware, "'.lower() != 'true' or '",
            enable_gripper_hardware, "'.lower() != 'true'",
        ])),
        parameters=[{
            'mobile_hardware_enabled': ParameterValue(
                enable_mobile_hardware,
                value_type=bool,
            ),
            'arm_hardware_enabled': ParameterValue(
                enable_arm_hardware,
                value_type=bool,
            ),
            'gripper_hardware_enabled': ParameterValue(
                enable_gripper_hardware,
                value_type=bool,
            ),
        }],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', full_controllers_yaml,
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', full_controllers_yaml,
        ],
        output='screen',
        condition=IfCondition(enable_arm_hardware),
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', full_controllers_yaml,
        ],
        output='screen',
        condition=IfCondition(enable_gripper_hardware),
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', full_controllers_yaml,
        ],
        output='screen',
        condition=IfCondition(enable_mobile_hardware),
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
            trajectory_execution,
            planning_scene_monitor,
            moveit_controllers,
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    moveit_pkg_path = get_package_share_directory(moveit_pkg)
    rviz_config = os.path.join(moveit_pkg_path, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz_arg),
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            world_to_odom,
            odom_to_base_footprint,
            robot_state_publisher,
            passive_joint_state_publisher,
            controller_manager,
            TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
            TimerAction(period=4.0, actions=[arm_controller_spawner]),
            TimerAction(period=5.0, actions=[gripper_controller_spawner]),
            TimerAction(period=6.0, actions=[diff_drive_controller_spawner]),
            TimerAction(period=8.0, actions=[move_group]),
            TimerAction(period=10.0, actions=[rviz_node]),
        ]
    )
