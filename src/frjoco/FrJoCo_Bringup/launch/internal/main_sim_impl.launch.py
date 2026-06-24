#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('joint_gui', default_value='false'),
        DeclareLaunchArgument('enable_web', default_value='true'),
        DeclareLaunchArgument('enable_move_executor', default_value='true'),
        DeclareLaunchArgument('enable_ee_pose_publisher', default_value='true'),
        DeclareLaunchArgument('enable_nav2_sim', default_value='true'),
        DeclareLaunchArgument('nav_rviz', default_value='true'),
        DeclareLaunchArgument('nav_init_x', default_value='0.0'),
        DeclareLaunchArgument('nav_init_y', default_value='0.0'),
        DeclareLaunchArgument('nav_init_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('planning_mode', default_value='manual_ik'),
        DeclareLaunchArgument('generated_orientation_mode', default_value='current'),
        DeclareLaunchArgument('orientation_tolerance', default_value='0.10'),
        DeclareLaunchArgument('workspace_gate_enabled', default_value='true'),
        DeclareLaunchArgument('workspace_min_x', default_value='-0.20'),
        DeclareLaunchArgument('workspace_max_x', default_value='0.80'),
        DeclareLaunchArgument('workspace_min_y', default_value='-0.55'),
        DeclareLaunchArgument('workspace_max_y', default_value='0.55'),
        DeclareLaunchArgument('workspace_min_z', default_value='0.02'),
        DeclareLaunchArgument('workspace_max_z', default_value='0.90'),
        DeclareLaunchArgument('workspace_min_xy_radius', default_value='0.05'),
        DeclareLaunchArgument('workspace_max_xy_radius', default_value='0.85'),
    ]

    publish_world_base_tf = PythonExpression([
        "'false' if '",
        LaunchConfiguration('enable_nav2_sim'),
        "'.lower() == 'true' else 'true'",
    ])

    planning_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mobile_manipulator_moveit_config'),
                'launch',
                'mobile_manipulator_planning_demo.launch.py',
            ])
        ),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'joint_gui': LaunchConfiguration('joint_gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level'),
            'publish_world_base_tf': publish_world_base_tf,
        }.items(),
    )

    nav2_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('frjoco_bringup'),
                'launch',
                'internal',
                'mobile_sim_impl.launch.py',
            ])
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('nav_rviz'),
            'init_x': LaunchConfiguration('nav_init_x'),
            'init_y': LaunchConfiguration('nav_init_y'),
            'init_yaw': LaunchConfiguration('nav_init_yaw'),
            'publish_robot_model': 'false',
            'use_composition': 'False',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_nav2_sim')),
    )

    web_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_web_interface'),
                'launch',
                'web_interface.launch.py',
            ])
        ),
        launch_arguments={
            'enable_gripper_bridge': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_web')),
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
            'planning_mode': LaunchConfiguration('planning_mode'),
            'use_input_orientation': False,
            'generated_orientation_mode': LaunchConfiguration('generated_orientation_mode'),
            'orientation_tolerance': ParameterValue(
                LaunchConfiguration('orientation_tolerance'),
                value_type=float,
            ),
            'workspace_gate_enabled': ParameterValue(
                LaunchConfiguration('workspace_gate_enabled'),
                value_type=bool,
            ),
            'workspace_min_x': ParameterValue(
                LaunchConfiguration('workspace_min_x'),
                value_type=float,
            ),
            'workspace_max_x': ParameterValue(
                LaunchConfiguration('workspace_max_x'),
                value_type=float,
            ),
            'workspace_min_y': ParameterValue(
                LaunchConfiguration('workspace_min_y'),
                value_type=float,
            ),
            'workspace_max_y': ParameterValue(
                LaunchConfiguration('workspace_max_y'),
                value_type=float,
            ),
            'workspace_min_z': ParameterValue(
                LaunchConfiguration('workspace_min_z'),
                value_type=float,
            ),
            'workspace_max_z': ParameterValue(
                LaunchConfiguration('workspace_max_z'),
                value_type=float,
            ),
            'workspace_min_xy_radius': ParameterValue(
                LaunchConfiguration('workspace_min_xy_radius'),
                value_type=float,
            ),
            'workspace_max_xy_radius': ParameterValue(
                LaunchConfiguration('workspace_max_xy_radius'),
                value_type=float,
            ),
            'require_confirmation': False,
            'home_on_start': False,
            'run_grasp_sequence': False,
            'execute_after_plan': False,
            'clear_rviz_after_execute': False,
            'status_topic': '/pick_place_status',
            'current_state_timeout': 20.0,
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'),
                value_type=bool,
            ),
        }],
        condition=IfCondition(LaunchConfiguration('enable_move_executor')),
    )

    ee_pose_publisher = Node(
        package='mobile_manipulator_moveit_config',
        executable='ee_pose_publisher.py',
        name='ee_pose_publisher',
        output='screen',
        parameters=[{
            'reference_frame': 'base_link',
            'left_finger_frame': 'gripper_left_link',
            'right_finger_frame': 'gripper_right_link',
            'fallback_frame': 'end_effector_link',
            'output_topic': '/gripper_center_pose',
            'publish_rate': 10.0,
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'),
                value_type=bool,
            ),
        }],
        condition=IfCondition(LaunchConfiguration('enable_ee_pose_publisher')),
    )

    return LaunchDescription(
        declared_arguments + [
            planning_sim,
            TimerAction(period=1.0, actions=[nav2_sim]),
            TimerAction(period=2.0, actions=[ee_pose_publisher]),
            TimerAction(period=3.0, actions=[vision_move]),
            TimerAction(period=1.0, actions=[web_interface]),
        ]
    )
