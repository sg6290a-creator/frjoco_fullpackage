'''
rtab_mapping.launch.py
============================================================================
Livox MID360 기반 RTAB-Map **매핑 전용** 런치 파일.

rtab_livox.launch.py 와의 차이점
- 기본 모드: mapping (localization=false) — 새 지도 생성 (시작 시 기존 DB 삭제)
- 기본 RViz ON: rtab_mapping.rviz 사용
  · Fixed Frame: map
  · MapCloud(/cloud_map)  → RTAB-Map 이 누적해 발행하는 3D 맵 클라우드
    (지나온 포인트클라우드가 keyframe 단위로 계속 누적/유지됨)
  · Map(/map)             → 2D occupancy grid
  · CurrentScan(/livox/lidar) → 현재 라이다 스캔
  · RobotModel, TF, Grid

세 개 런치 동시 실행 순서 (각각 별도 터미널)
    source /opt/ros/humble/setup.bash
    source ~/CAP_WS/install/setup.bash

    1) ros2 launch frjoco_bringup mobile_teleop.launch.py
    2) ros2 launch frjoco_bringup cam_lidar.launch.py
    3) ros2 launch frjoco_bringup rtab_mapping.launch.py

매핑 완료 후 DB 보존 예시
    mv ~/.ros/rtabmap.db ~/.ros/rtabmap_<장소>.db

옵션
- voxel_size:=0.1                 (다운샘플 복셀)
- expected_update_rate:=12.0      (Livox 프레임 레이트)
- qos:=2                          (Livox best effort)
- rviz:=false                     (RViz 끄고 헤드리스)
============================================================================
'''

import os
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
    rviz_enabled     = LaunchConfiguration('rviz').perform(context)
    rviz_enabled     = rviz_enabled == 'true' or rviz_enabled == 'True'

    rviz_config = os.path.join(
        get_package_share_directory('frjoco_bringup'),
        'rviz', 'rtab_mapping.rviz'
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
    # RTAB-Map 파라미터 (mapping 전용)
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
        'RGBD/CreateOccupancyGrid':       'true',
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
        # 누적 맵 클라우드(/cloud_map) 가 RViz 에서 잘 보이도록 보장
        'Rtabmap/PublishStats':           'true',
    }

    # mapping 모드: 시작 시 기존 DB 삭제
    arguments = ['-d']

    nodes = []

    # ICP Odometry
    nodes.append(
        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[shared_parameters, icp_odometry_parameters],
            remappings=[('scan_cloud', lidar_topic)],
        )
    )

    # RTAB-Map SLAM
    nodes.append(
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[shared_parameters, rtabmap_parameters],
            remappings=[('scan_cloud', lidar_topic)],
            arguments=arguments,
        )
    )

    # RViz2
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
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='시뮬레이션(Gazebo) 클럭 사용 여부'),

        DeclareLaunchArgument(
            'frame_id', default_value='base_footprint',
            description='로봇 기준 좌표계'),

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
            'rviz', default_value='true',
            description='RViz2 실행 여부 (mapping 용 기본 ON)'),

        OpaqueFunction(function=launch_setup),
    ])
