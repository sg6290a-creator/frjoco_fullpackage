#!/usr/bin/env python3
import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, file_path), 'r') as f:
        return yaml.safe_load(f)


def _load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, file_path), 'r') as f:
        return f.read()


def _launch_setup(context, *args, **kwargs):
    can_interface = LaunchConfiguration('can_interface').perform(context)
    ros_position_offset_wrist_3_joint = LaunchConfiguration(
        'ros_position_offset_wrist_3_joint'
    ).perform(context)
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    desc_pkg = get_package_share_directory('manipulator_description')
    ctrl_pkg = get_package_share_directory('rbdl_dls_controller')
    bringup_pkg = get_package_share_directory('frjoco_bringup')
    moveit_pkg = 'mobile_manipulator_moveit_config'

    urdf_path = os.path.join(desc_pkg, 'URDF', 'manipulator_urdf', 'ur5e_arm_only.urdf')
    with open(urdf_path, 'r') as f:
        robot_description_text = f.read().replace(
            '<param name="can_interface">can0</param>',
            f'<param name="can_interface">{can_interface}</param>',
        ).replace(
            '<param name="ros_position_offset_wrist_3_joint">0.0</param>',
            f'<param name="ros_position_offset_wrist_3_joint">{ros_position_offset_wrist_3_joint}</param>',
        )
    robot_description = {'robot_description': robot_description_text}

    controllers_yaml = os.path.join(ctrl_pkg, 'config', 'controllers.yaml')
    rviz_config = os.path.join(bringup_pkg, 'rviz', 'basic_display.rviz')

    kinematics_description = robot_description_text
    kin_yaml = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(
        {
            'dls_controller': {
                'ros__parameters': {
                    'kinematics_description': kinematics_description,
                    'end_effector_name': 'tool0',
                }
            }
        },
        kin_yaml,
    )
    kin_yaml.flush()

    robot_description_semantic = {
        'robot_description_semantic': _load_file(moveit_pkg, 'config/ur5e_arm_only.srdf')
    }
    robot_description_kinematics = {
        'robot_description_kinematics': _load_yaml(moveit_pkg, 'config/kinematics.yaml')
    }
    robot_description_planning = {
        'robot_description_planning': _load_yaml(moveit_pkg, 'config/joint_limits.yaml')
    }
    ompl_pipeline = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': _load_yaml(moveit_pkg, 'config/ompl_planning.yaml'),
    }
    controllers = _load_yaml(moveit_pkg, 'config/moveit_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': controllers.get(
            'moveit_simple_controller_manager', {}
        ),
        'moveit_controller_manager': controllers.get(
            'moveit_controller_manager',
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
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

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controllers_yaml, {'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            controllers_yaml,
        ],
    )

    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            controllers_yaml,
        ],
    )

    controller_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[joint_trajectory_controller],
        )
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
    )

    world_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        output='screen',
        arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
    )

    return [
        world_to_base,
        robot_state_publisher,
        controller_manager,
        TimerAction(period=2.0, actions=[joint_state_broadcaster]),
        controller_after_jsb,
        TimerAction(period=5.0, actions=[move_group]),
        TimerAction(period=8.0, actions=[rviz_node]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('ros_position_offset_wrist_3_joint', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_launch_setup),
    ])
