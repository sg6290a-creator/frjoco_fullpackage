#!/usr/bin/env python3
"""
Nav2 Launch (RTAB-Map /map 활용 — AMCL/map_server 없이)
===========================================================================
- 4번째 터미널에서 실행. 이미 떠있는 토픽/TF 위에 Nav2 만 얹는다.
- RTAB-Map 이 map→odom TF + /map (OccupancyGrid) 을 발행하므로
  nav2 의 amcl, map_server 는 띄우지 않는다 (navigation_launch.py 만 사용).

[전제 — 먼저 띄워둘 것]
  T1: ros2 launch frjoco_bringup mobile_teleop.launch.py
        → robot_state_publisher (URDF), diff_drive_controller,
          /diff_drive_controller/odom, /tf(base_footprint→…→mid360),
          /diff_drive_controller/cmd_vel_unstamped 구독
  T2: ros2 launch frjoco_bringup cam_lidar.launch.py
        → /livox/lidar (PointCloud2), 카메라, mid360→livox_frame TF
  T3: ros2 launch frjoco_bringup rtab_livox.launch.py
        → /map (OccupancyGrid, latched), map→odom TF,
          icp_odometry: odom→base_footprint TF

[본 launch 가 추가로 띄우는 것]
  1. pointcloud_to_laserscan
       /livox/lidar (PointCloud2)  →  /scan (LaserScan @ frame=livox_frame)
       (Nav2 의 voxel/obstacle layer 가 LaserScan 을 obstacle source 로 사용)
  2. nav2_bringup/navigation_launch.py
       controller_server, planner_server, behavior_server, bt_navigator,
       waypoint_follower, velocity_smoother, lifecycle_manager_navigation
  3. cmd_vel relay
       /cmd_vel  →  /diff_drive_controller/cmd_vel_unstamped
       (controller_server/velocity_smoother 출력은 /cmd_vel 인데,
        diff_drive_controller 는 /diff_drive_controller/cmd_vel_unstamped 를 구독)
  4. goal_pose relay
       /move_base_simple/goal  →  /goal_pose
       (RViz 기본 "2D Goal Pose" 툴이 /move_base_simple/goal 로 발행하므로)

[입력 토픽]
  /livox/lidar                              : sensor_msgs/PointCloud2 (cam_lidar)
  /map                                      : nav_msgs/OccupancyGrid  (rtab_livox, latched)
  /diff_drive_controller/odom               : nav_msgs/Odometry       (mobile_teleop)
  /tf, /tf_static                           : tf2_msgs/TFMessage      (모든 launch)

[출력 토픽]
  /scan                                     : sensor_msgs/LaserScan   (frame=livox_frame)
  /cmd_vel                                  : geometry_msgs/Twist     (controller→smoother)
  /diff_drive_controller/cmd_vel_unstamped  : geometry_msgs/Twist     (relay 결과)
  /local_costmap/costmap, /global_costmap/costmap, /plan ...

[RViz 사용법 (cam_lidar 의 RViz 또는 별도 RViz)]
  - Fixed Frame: map
  - Add → Map  (topic: /map)
  - Add → Path (topic: /plan)               # 전역 경로
  - Add → Costmap (topic: /local_costmap/costmap, /global_costmap/costmap)
  - 상단 툴바에서:
      "2D Pose Estimate"  : 초기 포즈 — RTAB-Map 사용 시 보통 자동이지만
                            오프셋이 있으면 /initialpose 발행 (AMCL 안 띄웠으므로
                            RTAB-Map 이 무시할 수 있음 — 그땐 로봇을 살짝 굴려서
                            ICP 가 정렬되게 하거나 rtab_livox localization 모드 사용).
      "Nav2 Goal"         : 목표 포즈 — /goal_pose 발행 → BT navigator 가 경로 생성.

[자주 쓰는 인자]
  params_file:=/path/to/your_nav2.yaml       # 파라미터 파일 변경
  scan_range_max:=10.0 scan_range_min:=0.3   # PC→LaserScan 변환 거리
  scan_height_min:=-0.2 scan_height_max:=0.2 # PC→LaserScan z 슬라이스 두께
  use_composition:=False                     # composable container 비활성
===========================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('frjoco_bringup')
    robot_nav2_pkg = get_package_share_directory('robot_nav2')
    nav2_pkg    = get_package_share_directory('nav2_bringup')

    default_params = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    nav2_launch    = os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')

    default_map  = os.path.join(robot_nav2_pkg, 'maps', 'my_map.yaml')
    default_rviz = os.path.join(bringup_pkg, 'rviz', 'nav2.rviz')

    args = [
        DeclareLaunchArgument('params_file',     default_value=default_params),
        DeclareLaunchArgument('map',             default_value=default_map),
        DeclareLaunchArgument('rviz_config',     default_value=default_rviz),
        DeclareLaunchArgument('use_sim_time',    default_value='false'),
        DeclareLaunchArgument('autostart',       default_value='true'),
        # navigation_launch.py 기본값과 동일 (False) — True 로 두면 별도 ComposableNode
        # 컨테이너를 띄워야 하는데 navigation_launch.py 는 컨테이너를 만들지 않음
        # → LoadComposableNodes 가 target 을 못 찾아 노드가 하나도 안 뜸.
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_respawn',     default_value='False'),
        DeclareLaunchArgument('log_level',       default_value='info'),

        # pointcloud_to_laserscan 슬라이스 파라미터
        DeclareLaunchArgument('scan_target_frame', default_value='livox_frame'),
        DeclareLaunchArgument('scan_range_min',    default_value='0.3'),
        DeclareLaunchArgument('scan_range_max',    default_value='15.0'),
        DeclareLaunchArgument('scan_height_min',   default_value='-0.15'),
        DeclareLaunchArgument('scan_height_max',   default_value='0.30'),
        DeclareLaunchArgument('scan_angle_min',    default_value='-3.14159'),
        DeclareLaunchArgument('scan_angle_max',    default_value='3.14159'),
        DeclareLaunchArgument('scan_angle_inc',    default_value='0.00872'),  # 0.5°
    ]

    # Livox PointCloud2 → LaserScan
    # Nav2 voxel/obstacle layer 가 LaserScan 을 직접 받음.
    pc_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame':    LaunchConfiguration('scan_target_frame'),
            'transform_tolerance': 0.05,
            'min_height':      LaunchConfiguration('scan_height_min'),
            'max_height':      LaunchConfiguration('scan_height_max'),
            'angle_min':       LaunchConfiguration('scan_angle_min'),
            'angle_max':       LaunchConfiguration('scan_angle_max'),
            'angle_increment': LaunchConfiguration('scan_angle_inc'),
            'scan_time':       0.1,
            'range_min':       LaunchConfiguration('scan_range_min'),
            'range_max':       LaunchConfiguration('scan_range_max'),
            'use_inf':         True,
            'inf_epsilon':     1.0,
            # Livox 는 best_effort
            'qos_overrides./livox/lidar.subscription.reliability': 'best_effort',
        }],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan',     '/scan'),
        ],
    )

    # /move_base_simple/goal  →  /goal_pose 릴레이
    # RViz 기본 "2D Goal Pose" 툴이 옛 토픽(/move_base_simple/goal)으로 발행하는데,
    # nav2 BT 는 /goal_pose 를 구독하므로 매번 RViz Tool Properties 에서 토픽
    # 바꾸는 대신 여기서 한 번에 릴레이.
    goal_relay = Node(
        package='topic_tools',
        executable='relay',
        name='goal_pose_relay',
        output='screen',
        arguments=['/move_base_simple/goal', '/goal_pose'],
    )

    # /cmd_vel  →  /diff_drive_controller/cmd_vel_unstamped (속도 cap 적용)
    # 단순 relay 가 아니라 cmd_vel_capper 노드로 교체.
    # 이유: nav2 의 behavior_server (BackUp / Spin / DriveOnHeading) 출력은
    #       velocity_smoother 를 우회해서 /cmd_vel 에 직접 publish 됨.
    #       → recovery 트리거 시 BT 가 0.025 m/s 같은 큰 후진 명령을 보내
    #         실제 로봇이 갑자기 빠르게 후진하는 위험 발생.
    #       capper 가 source 무관하게 |lin|≤0.01, |ang|≤0.02 로 강제 clip.
    cmd_vel_relay = Node(
        package='frjoco_bringup',
        executable='cmd_vel_capper.py',
        name='cmd_vel_capper',
        output='screen',
        parameters=[{
            'max_linear':  0.01,
            'max_angular': 0.02,
            'in_topic':    '/cmd_vel',
            'out_topic':   '/diff_drive_controller/cmd_vel_unstamped',
            'log_clipped': True,
        }],
    )

    # ─────────────────────────────────────────────────────────────────
    # 정적 4F 지도 publish
    # RTAB-Map 의 /map (계속 갱신되는 SLAM 지도 — RViz 에서 "더러운" 모습)
    # 대신 미리 저장된 my_map.yaml 을 /map_static 으로 publish.
    # nav2 의 costmap 은 (params 의 static_layer.map_topic 설정으로) 이 토픽을 구독.
    # RTAB-Map 의 /map 은 그대로 두지만 nav2 는 무시.
    # 위치추정(map→odom TF)은 RTAB-Map 이 계속 담당.
    # ─────────────────────────────────────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': LaunchConfiguration('map'),
            'topic_name': 'map_static',
            'frame_id': 'map',
        }],
    )

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

    # Nav2 navigation stack
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time':    LaunchConfiguration('use_sim_time'),
            'autostart':       LaunchConfiguration('autostart'),
            'params_file':     LaunchConfiguration('params_file'),
            'use_composition': LaunchConfiguration('use_composition'),
            'use_respawn':     LaunchConfiguration('use_respawn'),
            'log_level':       LaunchConfiguration('log_level'),
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription(
        args + [
            map_server, lifecycle_mgr_map,
            pc_to_scan, goal_relay, cmd_vel_relay,
            nav2, rviz,
        ]
    )
