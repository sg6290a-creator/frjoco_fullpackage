'''
입력 토픽 이름들 --> remapping에서 실제 topic name으로 변경
1. Livox MID360 포인트클라우드 'lidar_topic' (기본값: /livox/lidar)

출력 토픽 이름들
1. /icp_odom        → ICP Odometry
2. /map             → 2D occupancy grid (mapping 모드)
3. /rtabmap/mapData → RTAB-Map 3D 맵 데이터
'''

'''
이 런치 파일은 Livox MID360 3D LiDAR 기반으로 RTAB-Map(ICP Odometry + SLAM)을 구동합니다.
Livox 드라이버, EKF, Nav2 는 포함하지 않습니다.

출처: robot_bringup/launch/lidar3d_livox_nav.launch.py 에서 RTAB-Map 관련만 추출

사전 준비 (터미널마다 실행)
source install/setup.bash

함께 실행해야 하는 런치 (teleop, cam_lidar 먼저 켜고 rtab_livox 실행)
ros2 launch frjoco_bringup mobile_teleop.launch.py
ros2 launch frjoco_bringup cam_lidar.launch.py
ros2 launch frjoco_bringup rtab_livox.launch.py [모드 인자]

실행 명령어
1. mapping 모드 (새 지도 생성 — 시작 시 기존 ~/.ros/rtabmap.db 자동 삭제됨)
ros2 launch frjoco_bringup rtab_livox.launch.py

   맵핑 완료 후 DB 이름 변경하여 보존 (예시)
   mv ~/.ros/rtabmap.db ~/.ros/rtabmap_4F.db

2. localization 모드 (기존 DB로 위치 추정)★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
ros2 launch frjoco_bringup rtab_livox.launch.py \
  localization:=true \
  database_path:=~/.ros/rtabmap_4F.db

3. localization + AMCL 모드 (nav2.launch.py 와 함께 사용 — map→odom 은 AMCL 담당)
ros2 launch frjoco_bringup rtab_livox.launch.py \
  localization:=true \
  use_amcl:=true \
  database_path:=/home/ldh/.ros/rtabmap_4F.db

저장된 지도 목록
- 4층 : ~/.ros/rtabmap_4F.db
'''

import os
from pathlib import Path
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context: LaunchContext, *args, **kwargs):

    frame_id         = LaunchConfiguration('frame_id')
    voxel_size_value = float(LaunchConfiguration('voxel_size').perform(context))
    use_sim_time     = LaunchConfiguration('use_sim_time')
    lidar_topic      = LaunchConfiguration('lidar_topic').perform(context)
    localization     = LaunchConfiguration('localization').perform(context)
    localization     = localization == 'true' or localization == 'True'
    use_amcl         = LaunchConfiguration('use_amcl').perform(context)
    use_amcl         = use_amcl == 'true' or use_amcl == 'True'
    rviz_enabled     = LaunchConfiguration('rviz').perform(context)
    rviz_enabled     = rviz_enabled == 'true' or rviz_enabled == 'True'
    db_path          = LaunchConfiguration('database_path').perform(context)

    rviz_config = os.path.join(
        get_package_share_directory('frjoco_bringup'),
        'rviz', 'lidar3d_livox_nav.rviz'
    )

    # ================================================================
    # 공유 파라미터 (ICP Odometry + RTAB-Map 공통)
    # ================================================================
    shared_parameters = {
        'use_sim_time':                  use_sim_time,
        'frame_id':                      frame_id,
        'qos':                           LaunchConfiguration('qos'),
        'approx_sync':                   True,
        'wait_for_transform':            0.2,
        'Icp/PointToPlane':              'true',
        'Icp/Iterations':                '10',
        'Icp/VoxelSize':                 str(voxel_size_value),
        'Icp/Epsilon':                   '0.001',
        'Icp/PointToPlaneK':             '20',
        'Icp/PointToPlaneRadius':        '0',
        'Icp/MaxTranslation':            '3',
        'Icp/MaxCorrespondenceDistance': str(voxel_size_value * 10.0),
        'Icp/Strategy':                  '1',
        'Icp/OutlierRatio':              '0.7',
    }

    # ================================================================
    # ICP Odometry 파라미터
    # ================================================================
    icp_odometry_parameters = {
        'expected_update_rate':       LaunchConfiguration('expected_update_rate'),
        'deskewing':                  False,
        'odom_frame_id':              'odom',
        'publish_tf':                 True,
        'Odom/ScanKeyFrameThr':       '0.4',
        'OdomF2M/ScanSubtractRadius': str(voxel_size_value),
        'OdomF2M/ScanMaxSize':        '15000',
        'OdomF2M/BundleAdjustment':   'false',
        'Icp/CorrespondenceRatio':    '0.01',
    }

    # ================================================================
    # RTAB-Map 파라미터
    # ================================================================
    rtabmap_parameters = {
        'subscribe_depth':                False,
        'subscribe_rgb':                  False,
        'subscribe_scan_cloud':           True,
        'subscribe_odom_info':            True,
        'map_frame_id':                   'map',
        'odom_sensor_sync':               True,
        'map_always_update':              True,
        'map_empty_ray_tracing':          True,
        'RGBD/ProximityMaxGraphDepth':    '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate':             '0.05',
        'RGBD/LinearUpdate':              '0.05',
        'RGBD/CreateOccupancyGrid':       'false' if localization else 'true',
        'Reg/Strategy':                   '1',
        'Reg/Force3DoF':                  'true',
        # 3D 포인트클라우드 → 2D occupancy grid 변환
        'Grid/Sensor':                    '0',
        'Grid/3D':                        'true',
        'Grid/RangeMax':                  '20.0',
        'Grid/RangeMin':                  '0.5',
        'Grid/RayTracing':                'true',
        'Grid/CellSize':                  '0.05',
        'Grid/NormalsSegmentation':       'false',
        'Grid/MaxGroundHeight':           '0.0',
        'Grid/MaxObstacleHeight':         '2.0',
        'Grid/MinGroundHeight':           '-10.0',
        'Grid/ClusterRadius':             '0.1',
        'Grid/GroundIsObstacle':          'false',
        'Grid/FlatObstacleDetected':      'false',
        'Grid/NoiseFilteringRadius':      '0.0',
        'Grid/NoiseFilteringMinNeighbors':'0',
        'Grid/Scan2dUnknownSpaceFilled':  'true',
        'Mem/NotLinkedNodesKept':         'false',
        'Mem/STMSize':                    '30',
        'Icp/CorrespondenceRatio':        str(
            LaunchConfiguration('min_loop_closure_overlap').perform(context)
        ),
    }

    arguments = []
    if localization:
        rtabmap_parameters['Mem/IncrementalMemory'] = 'False'
        rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'True'
        if db_path:
            rtabmap_parameters['database_path'] = db_path
    else:
        arguments.append('-d')  # 시작 시 기존 DB 삭제

    if use_amcl:
        # AMCL이 map→odom TF를 담당하므로 RTAB-Map SLAM 노드의 TF 발행 중단
        rtabmap_parameters['publish_tf'] = False

    nodes = []

    # ================================================================
    # ICP Odometry
    # ================================================================
    nodes.append(
        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[shared_parameters, icp_odometry_parameters],
            remappings=[
                ('scan_cloud', lidar_topic),
            ]
        )
    )

    # ================================================================
    # RTAB-Map SLAM
    # ================================================================
    nodes.append(
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[shared_parameters, rtabmap_parameters],
            remappings=[('scan_cloud', lidar_topic)],
            arguments=arguments,
        )
    )

    # Static TF 불필요: cam_lidar.launch.py 가 mid360→livox_frame 을 이미 발행하고
    # URDF/RSP(teleop) 가 base_footprint→base_link→...→mid360 을 담당함.

    # ================================================================
    # RViz (선택)
    # ================================================================
    if rviz_enabled:
        nodes.append(
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        )

    return nodes


def generate_launch_description():
    default_database_path = str(Path.home() / '.ros' / 'rtabmap_4F.db')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='시뮬레이션(Gazebo) 클럭 사용 여부'),

        DeclareLaunchArgument(
            'frame_id', default_value='base_footprint',
            description='로봇 기준 좌표계'),

        DeclareLaunchArgument(
            'localization', default_value='true',
            description='true(기본): 기존 DB 로 위치 추정 (지도 갱신 OFF) / '
                        'false: 새 지도 생성 (DB 삭제 후 SLAM)'),

        DeclareLaunchArgument(
            'database_path', default_value=default_database_path,
            description='localization 모드에서 사용할 RTAB-Map DB 파일 경로 '
                        '(my_map.pgm 추출에 사용된 4F SLAM DB)'),

        DeclareLaunchArgument(
            'lidar_topic', default_value='/livox/lidar',
            description='Livox MID360 PointCloud2 토픽 이름'),

        DeclareLaunchArgument(
            'expected_update_rate', default_value='12.0',
            description='Livox 라이다 프레임 레이트 (실제 10Hz)'),

        DeclareLaunchArgument(
            'voxel_size', default_value='0.1',
            description='포인트클라우드 다운샘플 복셀 크기 (m)'),

        DeclareLaunchArgument(
            'min_loop_closure_overlap', default_value='0.2',
            description='루프 클로저 허용 최소 스캔 중복 비율'),

        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS: 0=시스템기본, 1=reliable, 2=best effort (Livox는 2 권장)'),

        DeclareLaunchArgument(
            'use_amcl', default_value='false',
            description='true: AMCL이 map→odom TF 담당 → RTAB-Map TF 발행 중단'),

        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='RViz2 실행 여부'),

        OpaqueFunction(function=launch_setup),
    ])
