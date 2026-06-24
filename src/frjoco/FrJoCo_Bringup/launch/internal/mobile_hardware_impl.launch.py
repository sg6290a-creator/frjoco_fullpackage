#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port_front = LaunchConfiguration('port_front')
    port_rear = LaunchConfiguration('port_rear')
    front_driver_id = LaunchConfiguration('front_driver_id')
    rear_driver_id = LaunchConfiguration('rear_driver_id')
    mobile_baudrate = LaunchConfiguration('mobile_baudrate')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_web = LaunchConfiguration('enable_web')
    rviz = LaunchConfiguration('rviz')
    nav_rviz = LaunchConfiguration('nav_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    nav_rviz_config = LaunchConfiguration('nav_rviz_config')
    publish_world_odom_tf = LaunchConfiguration('publish_world_odom_tf')

    urdf_path = PathJoinSubstitution([
        FindPackageShare('frjoco_model'),
        'URDF',
        'mobile_urdf',
        'mobile_hardware.urdf.xacro',
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', urdf_path, ' ',
                'port_front:=', port_front, ' ',
                'port_rear:=', port_rear, ' ',
                'front_driver_id:=', front_driver_id, ' ',
                'rear_driver_id:=', rear_driver_id, ' ',
                'mobile_baudrate:=', mobile_baudrate,
            ]),
            value_type=str,
        )
    }

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('frjoco_bringup'),
        'config',
        'diff_drive_controller.yaml',
    ])

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
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml,
        ],
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml,
        ],
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
        condition=IfCondition(enable_web),
    )

    mobile_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='mobile_only_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    navigation_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='mobile_nav_rviz',
        output='screen',
        arguments=['-d', nav_rviz_config],
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        condition=IfCondition(nav_rviz),
    )

    return LaunchDescription([
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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_web', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('nav_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('frjoco_bringup'),
                'rviz',
                'basic_display.rviz',
            ]),
        ),
        DeclareLaunchArgument(
            'nav_rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('frjoco_bringup'),
                'rviz',
                'nav2.rviz',
            ]),
        ),
        DeclareLaunchArgument('publish_world_odom_tf', default_value='true'),
        world_to_odom,
        robot_state_publisher,
        controller_manager,
        TimerAction(period=2.0, actions=[joint_state_broadcaster]),
        TimerAction(period=3.0, actions=[diff_drive_controller]),
        TimerAction(period=4.0, actions=[mobile_rviz]),
        TimerAction(period=5.0, actions=[navigation_rviz]),
        TimerAction(period=5.0, actions=[web_interface]),
    ])
