#!/usr/bin/env python3
"""
Camera + Lidar Combined Launch (RealSense D455/D405 + Livox MID360)
===========================================================================
- cameras.launch.py + lidar.launch.py 의 핵심 노드를 한번에 띄움
- RViz2: lidar.launch.py 와 동일하게 livox 의 display_point_cloud_ROS2.rviz 사용
        → /livox/lidar 만 자동 표시 (안정성을 위해 Image display 는 미리 박지 않음).
        영상이 보고 싶으면 RViz UI 에서 [Add → By topic] 으로 원하는 카메라
        토픽의 Image display 를 1~2개 직접 추가하면 됨.
        (4개를 미리 config 에 박으면 일부 GL 드라이버에서 libGL drawable 실패 →
         SIGSEGV 가 발생함 — 사용자 환경에서 재현됨.)
- 카메라 토픽 수신 여부는 launch 후 10초 시점의 🚦 신호등으로 확인
  (RViz 안에 영상을 미리 박는 대신 토픽이 살아있는지만 보장).

[입력 토픽]
  (없음) — USB / 192.168.1.160(UDP) 직접 수신

[출력 토픽]
  ── 카메라 (cameras.launch.py 와 동일) ─────────────────────────────────
    /d455/color/image_raw, /d455/aligned_depth_to_color/image_raw,
    /d455/depth/color/points, /d455/depth/camera_info ...
    /d405/color/image_raw, /d405/aligned_depth_to_color/image_raw,
    /d405/depth/color/points, /d405/depth/camera_info ...
    ※ D405 RGB는 image_rect_raw (image_raw 미발행 — D455 와 다름)
    ※ D455 IMU 비활성화 (livox/imu 사용 + iio sysfs 권한 이슈)

  ── 라이다 (lidar.launch.py 와 동일) ───────────────────────────────────
    /livox/lidar  : sensor_msgs/PointCloud2  (~10Hz, frame_id=livox_frame)
    /livox/imu    : sensor_msgs/Imu          (~200Hz, frame_id=livox_frame)
    /tf_static    : livox_frame -> livox_imu_frame  (x=0.011, y=0.02329, z=-0.04412)

       ── 본 launch가 소유하는 frame ──
         livox_frame, livox_imu_frame
         d455_link, d455_color_frame, d455_color_optical_frame,
         d455_depth_frame, d455_depth_optical_frame ...
         d405_link, d405_color_frame, d405_color_optical_frame,
         d405_depth_frame, d405_depth_optical_frame ...
       ※ 카메라/라이다 frame 사이에는 본 launch가 정적 TF를 발행하지 않음 —
          robot URDF (mobile_teleop.launch.py) 또는 sensor_setup.launch.py 와
          함께 띄울 때 그쪽이 d455_link/d405_link/livox_frame 을 base_link 로
          묶어 줘야 한 좌표계에서 보임. 단독 점검 시 RViz Fixed Frame 를
          livox_frame 으로 두면 PointCloud 만 정상 표시.
===========================================================================
[최초 1회 설치 / 코드 변경 후 재빌드]
  sudo apt install ros-humble-librealsense2* ros-humble-realsense2-camera
  cd ~/CAP_WS
  colcon build --packages-select livox_ros_driver2 frjoco_bringup --symlink-install
  source install/setup.bash

[실행 절차]

1. 잔여 프로세스 종료 (선택)
   pkill -f realsense2_camera_node
   pkill -f livox_ros_driver2_node
   pkill -f rviz2

2. 연결 점검
   rs-enumerate-devices -s         # D455/D405 두 줄 다 떠야 정상
   ping -c 3 192.168.1.160         # MID360 응답 확인
   ip addr show | grep 192.168.1   # 호스트 NIC 192.168.1.5/24 확인

3. ROS2 환경 소싱 / launch
   source /opt/ros/humble/setup.bash
   source ~/CAP_WS/install/setup.bash
   ros2 launch frjoco_bringup cam_lidar.launch.py

   터미널에 다음 세 배너가 순서대로 뜨면 정상:

     ╔══════════════════════════════════════════════════════════╗
     ║         📷  RealSense 연결 상태 확인 (USB enum)           ║
     ╚══════════════════════════════════════════════════════════╝
       ✅ D455 : 연결완료!  →  serial=2136xxxxxxxx
       ✅ D405 : 연결완료!  →  serial=3151xxxxxxxx

     (livox 드라이버 stdout)
       ✅ livox/lidar publish use PointCloud2 format
       ✅ livox/imu   publish use imu format

     (~10초 후)
     ╔══════════════════════════════════════════════════════════╗
     ║         🚦  센서 토픽 신호등 (카메라 + 라이다)            ║
     ╚══════════════════════════════════════════════════════════╝
       🟢 📡 Livox PointCloud  /livox/lidar
       🟢 🧭 Livox IMU         /livox/imu
       🟢 🎨 D455 RGB          /d455/color/image_raw
       🟢 📏 D455 Depth        /d455/aligned_depth_to_color/image_raw
       🟢 🎨 D405 RGB          /d405/color/image_raw
       🟢 📏 D405 Depth        /d405/aligned_depth_to_color/image_raw

   - ❌ (USB enum)        → USB3 포트/케이블, rs-viewer 등 다른 데몬 종료
   - 🔴 (라이다 신호등)    → 192.168.1.160 ping / 호스트 IP / MID360_config.json
   - 🔴 (카메라 신호등)    → D405는 image_rect_raw 가 맞는지 / 노드 부팅 지연

4. RViz 사용
   - Fixed Frame: livox_frame (livox 기본 config 그대로)
   - PointCloud2 가 자동으로 /livox/lidar 를 구독 — 라이다 주변에 점이 회전하며
     찍히면 정상.
   - 카메라 영상을 같이 보고 싶으면 RViz 좌하단 [Add] → By topic 에서
       /d455/color/image_raw         → Image
       /d455/aligned_depth_to_color/image_raw   → Image
       /d405/color/image_raw         → Image
       /d405/aligned_depth_to_color/image_raw   → Image
     중 원하는 것만 추가. (한 번에 1~2개만 켜기를 권장 — 다수 동시 enable 시
     일부 환경에서 GL drawable 실패로 RViz 가 죽을 수 있음.)

[자주 쓰는 인자]
  enable_rviz:=false                          # RViz 끄기 (헤드리스 점검)
  d455_serial:="'213622301251'"               # 자동 탐지 무시
  d405_serial:="'315122271488'"
  publish_freq:=20.0                          # Livox 출력 주기 (5/10/20/50)
  xfer_format:=1                              # Livox custom 포맷 (RViz 비호환)
  rviz_config:=/path/to/other.rviz            # 다른 RViz 설정 사용
===========================================================================
"""

import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _probe_realsense():
    """rs-enumerate-devices -s 로 연결된 카메라 (모델, 시리얼) 매핑."""
    try:
        out = subprocess.run(
            ['rs-enumerate-devices', '-s'],
            capture_output=True, text=True, timeout=5.0,
        ).stdout
    except Exception:
        return {}

    mapping = {}
    pat = re.compile(r'Intel RealSense\s+(\S+)\s+(\S+)\s+\S+')
    for line in out.splitlines():
        m = pat.search(line)
        if m:
            model, serial = m.group(1), m.group(2)
            mapping[model] = serial
    return mapping


def generate_launch_description():
    rs_share  = get_package_share_directory('realsense2_camera')
    rs_multi  = os.path.join(rs_share, 'launch', 'rs_multi_camera_launch.py')
    rs_single = os.path.join(rs_share, 'launch', 'rs_launch.py')
    livox_pkg = get_package_share_directory('livox_ros_driver2')
    # frjoco_bringup/rviz/cam_lidar.rviz : livox 기본 config 를 복사 + 2D Goal Pose
    # 의 Topic 만 /move_base_simple/goal → /goal_pose 로 변경 (Nav2 표준 토픽).
    # → RViz Tool Properties 에서 매번 수동으로 토픽 바꿀 필요 없음.
    default_rviz = os.path.join(
        get_package_share_directory('frjoco_bringup'), 'rviz', 'cam_lidar.rviz'
    )

    # ── 카메라 USB 탐지 + 배너 ─────────────────────────────────────────────
    probed = _probe_realsense()
    d455 = probed.get('D455')
    d405 = probed.get('D405')

    print('')
    print('╔══════════════════════════════════════════════════════════╗')
    print('║         📷  RealSense 연결 상태 확인 (USB enum)           ║')
    print('╚══════════════════════════════════════════════════════════╝')
    if d455 is not None:
        print(f'  ✅ D455 : 연결완료!  →  serial={d455}')
    else:
        print('  ❌ D455 : 응답 없음 — USB3 포트/케이블/전원 확인')
    if d405 is not None:
        print(f'  ✅ D405 : 연결완료!  →  serial={d405}')
    else:
        print('  ❌ D405 : 응답 없음 — USB3 포트/케이블/전원 확인')
    if d455 is None and d405 is None:
        print('')
        print('  ⚠️  카메라가 하나도 감지되지 않았습니다 — 카메라 노드는 띄우지 않습니다.')
        print('     (라이다는 정상 진행)')
    print('')

    # ── Arguments ──────────────────────────────────────────────────────────
    args = [
        # 카메라
        DeclareLaunchArgument(
            'd455_serial',
            default_value=(f"'{d455}'" if d455 else "''"),
            description="D455 시리얼 — 따옴표 포함 형식 (예: \"'213622301251'\")",
        ),
        DeclareLaunchArgument(
            'd405_serial',
            default_value=(f"'{d405}'" if d405 else "''"),
            description="D405 시리얼 — 따옴표 포함 형식",
        ),
        # 라이다
        DeclareLaunchArgument('xfer_format',      default_value='0'),
        DeclareLaunchArgument('multi_topic',      default_value='0'),
        DeclareLaunchArgument('data_src',         default_value='0'),
        DeclareLaunchArgument('publish_freq',     default_value='10.0'),
        DeclareLaunchArgument('output_data_type', default_value='0'),
        DeclareLaunchArgument('frame_id',         default_value='livox_frame'),
        DeclareLaunchArgument('cmdline_bd_code',  default_value='livox0000000001'),
        DeclareLaunchArgument(
            'user_config_path',
            default_value=os.path.join(livox_pkg, 'config', 'MID360_config.json'),
            description='MID360 네트워크/외부 파라미터 JSON 경로',
        ),
        DeclareLaunchArgument(
            'lvx_file_path',
            default_value='/home/livox/livox_test.lvx',
            description='lvx 파일 경로 (data_src=lidar 일 때는 사용되지 않음)',
        ),
        # RViz
        # 기본 false: 전체 stack 운용 시엔 nav2.launch.py 의 nav2.rviz 만 띄움.
        # 카메라/라이다 단독 점검 시에만 enable_rviz:=true 로 켜기.
        DeclareLaunchArgument('enable_rviz', default_value='false',
                              description='cam_lidar 단독 점검용 RViz2 (기본 OFF)'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz2 설정 파일 경로 (기본: livox display_point_cloud_ROS2.rviz)',
        ),
    ]

    actions = []

    # ── 카메라 노드 (D455 / D405) ─────────────────────────────────────────
    if d455 is not None and d405 is not None:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_multi),
            launch_arguments={
                'camera_name1':                 'd455',
                'camera_namespace1':            '',
                'serial_no1':                   LaunchConfiguration('d455_serial'),
                'enable_gyro1':                 'false',
                'enable_accel1':                'false',
                'enable_depth1':                'false',
                'align_depth.enable1':          'false',
                'pointcloud.enable1':           'false',
                'rgb_camera.color_profile1':    '424x240x30',

                'camera_name2':                 'd405',
                'camera_namespace2':            '',
                'serial_no2':                   LaunchConfiguration('d405_serial'),
                'align_depth.enable2':          'true',
                'pointcloud.enable2':           'false',
                'depth_module.color_profile2':  '848x480x30',
            }.items(),
        ))
    elif d455 is not None:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_single),
            launch_arguments={
                'camera_name':                'd455',
                'camera_namespace':           '',
                'serial_no':                  LaunchConfiguration('d455_serial'),
                'enable_gyro':                'false',
                'enable_accel':               'false',
                'align_depth.enable':         'true',
                'pointcloud.enable':          'false',
                'rgb_camera.color_profile':   '424x240x30',
            }.items(),
        ))
    elif d405 is not None:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_single),
            launch_arguments={
                'camera_name':                  'd405',
                'camera_namespace':             '',
                'serial_no':                    LaunchConfiguration('d405_serial'),
                'align_depth.enable':           'true',
                'pointcloud.enable':            'false',
                'depth_module.color_profile':   '848x480x30',
            }.items(),
        ))

    # ── Livox driver + static TF ──────────────────────────────────────────
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format':           LaunchConfiguration('xfer_format'),
            'multi_topic':           LaunchConfiguration('multi_topic'),
            'data_src':              LaunchConfiguration('data_src'),
            'publish_freq':          LaunchConfiguration('publish_freq'),
            'output_data_type':      LaunchConfiguration('output_data_type'),
            'frame_id':              LaunchConfiguration('frame_id'),
            'lvx_file_path':         LaunchConfiguration('lvx_file_path'),
            'user_config_path':      LaunchConfiguration('user_config_path'),
            'cmdline_input_bd_code': LaunchConfiguration('cmdline_bd_code'),
        }],
    )
    # MID360 사양서 외부 파라미터 (IMU 원점 - LiDAR 원점, 단위 m)
    livox_imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_imu_static_tf',
        output='screen',
        arguments=[
            '--x', '0.011', '--y', '0.02329', '--z', '-0.04412',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'livox_frame',
            '--child-frame-id', 'livox_imu_frame',
        ],
    )
    # ── URDF ↔ 드라이버 frame 브릿지 (identity static TF 2개) ─────────────
    # URDF 의 camera_link / d405 / mid360 와 드라이버가 만드는
    # d455_link / d405_link / livox_frame 은 같은 물리 원점이라 identity 로 충분.
    # 이게 없으면 RViz 에서 robot_description 을 띄울 때 base_link → livox_frame
    # 변환이 끊겨서 RobotModel 이 빨갛게 표시됨.
    # (cam_lidar 단독 + URDF 미로드 환경에서도 무해 — 그저 dangling edge.)
    tf_camera_bridge = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_camera_link_to_d455_link', output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'd455_link',
        ],
    )
    tf_d405_bridge = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_d405_to_d405_link', output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'd405',
            '--child-frame-id', 'd405_link',
        ],
    )
    tf_lidar_bridge = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_mid360_to_livox_frame', output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'mid360',
            '--child-frame-id', 'livox_frame',
        ],
    )

    actions += [
        livox_driver,
        livox_imu_static_tf,
        tf_camera_bridge,
        tf_d405_bridge,
        tf_lidar_bridge,
    ]

    # ── 통합 RViz2 ────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='cam_lidar_rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )
    actions.append(rviz_node)

    # ── 통합 신호등 (카메라 + 라이다) ──────────────────────────────────────
    # 라이다는 무조건 체크, 카메라는 USB enum 결과에 따라 추가.
    streams = [
        ('📡 Livox PointCloud', '/livox/lidar'),
        ('🧭 Livox IMU       ', '/livox/imu'),
    ]
    if d455 is not None:
        streams.append(('🎨 D455 RGB        ', '/d455/color/image_raw'))
        streams.append(('📏 D455 Depth      ', '/d455/aligned_depth_to_color/image_raw'))
    if d405 is not None:
        streams.append(('🎨 D405 RGB        ', '/d405/color/image_raw'))
        streams.append(('📏 D405 Depth      ', '/d405/aligned_depth_to_color/image_raw'))

    # 카메라 노드 부팅이 라이다보다 느림 → 15초로 여유 확보
    head = (
        'echo ""; '
        "echo '╔══════════════════════════════════════════════════════════╗'; "
        "echo '║         🚦  센서 토픽 신호등 (카메라 + 라이다)            ║'; "
        "echo '╚══════════════════════════════════════════════════════════╝'; "
        'TMP=$(mktemp -d); '
    )
    # --qos-reliability best_effort: best_effort sub 는 reliable pub 와도 매칭되므로
    # 라이다(reliable) / realsense(reliable) / 혹시 best_effort 인 토픽 전부 커버.
    # 6개 병렬 시 DDS 디스커버리 경합 → 타임아웃 8초로 여유.
    parallel = []
    for idx, (label, topic) in enumerate(streams):
        parallel.append(
            f"(if timeout 8 ros2 topic echo --once --qos-reliability best_effort "
            f"{topic} >/dev/null 2>&1; "
            f"then echo '  🟢 {label}  {topic}' > $TMP/{idx}; "
            f"else echo '  🔴 {label}  {topic}  (수신 실패)' > $TMP/{idx}; fi) &"
        )
    tail = (
        ' wait; for i in $(seq 0 ' + str(len(streams) - 1) + '); '
        'do cat $TMP/$i; done; rm -rf $TMP; echo ""'
    )
    full_cmd = head + ' '.join(parallel) + tail
    actions.append(TimerAction(
        period=15.0,
        actions=[ExecuteProcess(cmd=['bash', '-c', full_cmd], output='screen')],
    ))

    return LaunchDescription(args + actions)
