#!/usr/bin/env python3
"""
mobile_sim.launch.py — 4F 정적 지도 위에서 nav2 경로계획/추종 단독 검증용 시뮬레이션

실 센서 / RTAB-Map / 모터 / URDF 없이 다음만 띄움:

  1. map_server                       — 4F PGM 지도 publish (/map)
  2. lifecycle_manager (map_server)   — map_server activate
  3. sim_robot.py                     — 가상 diff-drive (cmd_vel 적분 → odom + TF)
                                         + map → odom identity TF
  4. nav2_bringup/navigation_launch   — planner / controller(RPP) / bt / behavior
  5. cmd_vel_capper                   — 한계 속도 강제 clip (실기와 동일)
  6. RViz2

사용법
  ros2 launch frjoco_bringup mobile_sim.launch.py
  # 다른 지도/초기 위치 지정
  ros2 launch frjoco_bringup mobile_sim.launch.py \\
      map:=/path/to/other.yaml init_x:=2.0 init_y:=-3.0

[RViz 사용]
  - Fixed Frame: map (자동 설정됨)
  - "Nav2 Goal" 버튼으로 목표 클릭 → /goal_pose 발행
  - sim_robot 가 /cmd_vel 받아 가상 이동 → 경로/추종 시각화

[입력 토픽] (없음 — 가상 환경)
[출력 토픽]
  /map                                       (map_server)
  /diff_drive_controller/cmd_vel_unstamped   (capper 출력 — 실기와 동일)
  /diff_drive_controller/odom                (sim_robot)
  /tf, /tf_static                            (sim_robot + nav2)
  /plan, /local_costmap/costmap, /global_costmap/costmap, /cmd_vel ...
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_pkg = get_package_share_directory('frjoco_bringup')
    robot_nav2_pkg = get_package_share_directory('robot_nav2')
    nav2_pkg    = get_package_share_directory('nav2_bringup')

    default_map    = os.path.join(robot_nav2_pkg, 'maps', 'my_map.yaml')
    default_params = os.path.join(bringup_pkg, 'config', 'sim_nav2_params.yaml')
    default_rviz   = os.path.join(bringup_pkg, 'rviz', 'sim_nav2.rviz')
    nav2_launch    = os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')

    urdf_path = PathJoinSubstitution([
        FindPackageShare('manipulator_description'),
        'URDF',
        'mobile_manipulator_urdf',
        'mobile_manipulator_6dof_gripper.urdf.xacro',
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_path, ' ',
        'enable_mobile_hardware:=false ',
        'enable_gripper_hardware:=false',
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }

    args = [
        DeclareLaunchArgument('map',         default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('use_rviz',    default_value='true'),
        DeclareLaunchArgument('publish_robot_model', default_value='true'),

        # 초기 spawn 위치 (map frame)
        DeclareLaunchArgument('init_x',   default_value='0.0'),
        DeclareLaunchArgument('init_y',   default_value='0.0'),
        DeclareLaunchArgument('init_yaw', default_value='0.0'),

        DeclareLaunchArgument('autostart',       default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
    ]

    # ---- map_server ---------------------------------------------------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': LaunchConfiguration('map'),
            'topic_name': 'map',
            'frame_id': 'map',
        }],
    )

    # map_server 는 lifecycle node — 별도 manager 로 activate
    lifecycle_mgr_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # ---- 가상 로봇 -----------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_robot_model')),
        parameters=[
            robot_description,
            {'use_sim_time': False},
        ],
    )

    joint_state_publisher = Node(
        package='mobile_manipulator_moveit_config',
        executable='demo_joint_state_publisher.py',
        name='demo_joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_robot_model')),
        parameters=[
            {'use_sim_time': False},
        ],
    )

    sim_robot = Node(
        package='frjoco_bringup',
        executable='sim_robot.py',
        name='sim_robot',
        output='screen',
        parameters=[{
            'init_x':   LaunchConfiguration('init_x'),
            'init_y':   LaunchConfiguration('init_y'),
            'init_yaw': LaunchConfiguration('init_yaw'),
            'rate': 50.0,
            'cmd_topic':  '/diff_drive_controller/cmd_vel_unstamped',
            'odom_topic': '/diff_drive_controller/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'map_frame':  'map',
            'publish_map_odom_tf': True,
        }],
    )

    # ---- cmd_vel capper (SIM 전용 — 10x 한도) -------------------------------
    # 실기 launch (nav2.launch.py) 는 0.01 / 0.02 그대로 유지.
    # 본 launch 의 sim_nav2_params.yaml 도 10x 로 맞춰져 있어서
    # capper 가 실제로 clip 하지 않고 그대로 통과하지만, 안전 상한으로 둠.
    cmd_vel_capper = Node(
        package='frjoco_bringup',
        executable='cmd_vel_capper.py',
        name='cmd_vel_capper',
        output='screen',
        parameters=[{
            'max_linear':  2.5,
            'max_angular': 5.0,
            'in_topic':    '/cmd_vel',
            'out_topic':   '/diff_drive_controller/cmd_vel_unstamped',
            'log_clipped': True,
        }],
    )

    # ---- goal_pose relay (RViz "2D Goal Pose" 호환) ------------------------
    goal_relay = Node(
        package='topic_tools',
        executable='relay',
        name='goal_pose_relay',
        output='screen',
        arguments=['/move_base_simple/goal', '/goal_pose'],
    )

    # ---- nav2 stack ---------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time':    'false',
            'autostart':       LaunchConfiguration('autostart'),
            'params_file':     LaunchConfiguration('params_file'),
            'use_composition': LaunchConfiguration('use_composition'),
            'use_respawn':     'False',
            'log_level':       'info',
        }.items(),
    )

    # ---- RViz ---------------------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(args + [
        robot_state_publisher,
        joint_state_publisher,
        map_server,
        lifecycle_mgr_map,
        sim_robot,
        cmd_vel_capper,
        goal_relay,
        nav2,
        rviz,
    ])
