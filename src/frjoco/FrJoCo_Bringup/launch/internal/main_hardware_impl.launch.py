#!/usr/bin/env python3
"""
Arm operation demo launch.

This is the top-level runtime entrypoint for the current UI-driven grasp flow:

1. Integrated mobile-manipulator hardware + MoveIt
2. RealSense D405 + D455 image/depth streams
3. YOLO segmentation and thin-part grasp estimator
4. YOLO grasp -> MoveIt PoseStamped bridge
5. vision_move executor
6. Web UI

The mobile base is included in the robot model/controller stack, but MoveIt plans
only the arm group. The UI still publishes mobile velocity commands directly to
the diff_drive_controller when needed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('default_velocity', default_value='0.25'),
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
        DeclareLaunchArgument('enable_realsense', default_value='true'),
        DeclareLaunchArgument('d455_serial', default_value="'213622301251'"),
        DeclareLaunchArgument('d405_serial', default_value="'315122271488'"),
        DeclareLaunchArgument('d455_color_profile', default_value='424x240x30'),
        DeclareLaunchArgument('d405_color_profile', default_value='848x480x30'),
        DeclareLaunchArgument('enable_yolo', default_value='true'),
        DeclareLaunchArgument('enable_web', default_value='true'),
        DeclareLaunchArgument('enable_mobile_teleop', default_value='false'),
        DeclareLaunchArgument(
            'mobile_teleop_topic',
            default_value='/diff_drive_controller/cmd_vel_unstamped',
        ),
        DeclareLaunchArgument('mobile_teleop_speed', default_value='0.08'),
        DeclareLaunchArgument('mobile_teleop_turn', default_value='0.25'),
        DeclareLaunchArgument('enable_move_executor', default_value='true'),
        DeclareLaunchArgument('enable_ee_pose_publisher', default_value='true'),
        DeclareLaunchArgument('enable_hardware_status', default_value='true'),
        DeclareLaunchArgument('enable_nav2', default_value='false'),
        DeclareLaunchArgument('enable_nav_sensors', default_value='false'),
        DeclareLaunchArgument('enable_rtab_livox', default_value='false'),
        DeclareLaunchArgument('nav_localization', default_value='true'),
        DeclareLaunchArgument('nav_scan_target_frame', default_value='livox_frame'),
        DeclareLaunchArgument('require_confirmation', default_value='false'),
        DeclareLaunchArgument('home_on_start', default_value='false'),
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
        DeclareLaunchArgument('place_transition_mode', default_value='direct'),
        DeclareLaunchArgument('load_turn_joint_name', default_value='shoulder_pan_joint'),
        DeclareLaunchArgument('load_turn_angle', default_value='1.5708'),
        DeclareLaunchArgument('carry_elbow_extra_bend_deg', default_value='6.0'),
        DeclareLaunchArgument('pre_grasp_align_enabled', default_value='true'),
        DeclareLaunchArgument('pre_grasp_align_x_offset', default_value='-0.10'),
        DeclareLaunchArgument('pre_grasp_align_settle_sec', default_value='0.7'),
        DeclareLaunchArgument('grasp_roll_enabled', default_value='true'),
        DeclareLaunchArgument('grasp_roll_joint_name', default_value='wrist_3_joint'),
        DeclareLaunchArgument('grasp_roll_sign', default_value='1.0'),
        DeclareLaunchArgument('grasp_roll_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('grasp_roll_max_abs_deg', default_value='90.0'),
        DeclareLaunchArgument('grasp_roll_max_age_sec', default_value='5.0'),
        DeclareLaunchArgument('hold_manual_gripper_open', default_value='true'),
        DeclareLaunchArgument('gripper_hold_joint_name', default_value='gripper_left_joint'),
        DeclareLaunchArgument('post_place_turn_enabled', default_value='true'),
        DeclareLaunchArgument('post_place_turn_joint_name', default_value='shoulder_pan_joint'),
        DeclareLaunchArgument('post_place_turn_angle', default_value='-3.1416'),
        DeclareLaunchArgument('pre_open_after_place_delay_sec', default_value='1.5'),
        DeclareLaunchArgument('place_shoulder_pan', default_value='0.0'),
        DeclareLaunchArgument('place_shoulder_lift', default_value='-0.0700'),
        DeclareLaunchArgument('place_elbow', default_value='-2.4870'),
        DeclareLaunchArgument('place_wrist_1', default_value='1.3538'),
        DeclareLaunchArgument('place_wrist_2', default_value='0.0'),
        DeclareLaunchArgument('place_wrist_3', default_value='0.0'),
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/frlab/Downloads/yolo_26/result/sam3_finetune_rot50_e20_plus80_retry/weights/best.pt',
        ),
        DeclareLaunchArgument(
            'yolo_pythonpath',
            default_value='/home/frlab/miniconda3/envs/ultradex/lib/python3.10/site-packages',
        ),
        DeclareLaunchArgument('color_topic', default_value='auto'),
        DeclareLaunchArgument('depth_topic', default_value='auto'),
        DeclareLaunchArgument('camera_info_topic', default_value='auto'),
        DeclareLaunchArgument('camera_frame', default_value='d405_optical_frame'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.5'),
        DeclareLaunchArgument('target_class_id', default_value='-1'),
        DeclareLaunchArgument('yolo_imgsz', default_value='672'),
        DeclareLaunchArgument('yolo_retina_masks', default_value='true'),
        DeclareLaunchArgument('yolo_max_inference_hz', default_value='8.0'),
        DeclareLaunchArgument('yolo_preview_publish_hz', default_value='10.0'),
        DeclareLaunchArgument('show_yolo_visualization', default_value='false'),
        DeclareLaunchArgument('enable_keyboard_trigger', default_value='false'),
    ]

    controllers_file = PythonExpression([
        "'mobile_manipulator_nav_controllers.yaml' if '",
        LaunchConfiguration('enable_nav2'),
        "'.lower() == 'true' else 'mobile_manipulator_full_controllers.yaml'",
    ])
    publish_world_odom_tf = PythonExpression([
        "'false' if '",
        LaunchConfiguration('enable_nav2'),
        "'.lower() == 'true' else 'true'",
    ])
    vision_color_topic = PythonExpression([
        "'/d405/color/image_raw' if '",
        LaunchConfiguration('color_topic'),
        "'.lower() == 'auto' else '",
        LaunchConfiguration('color_topic'),
        "'",
    ])
    vision_depth_topic = PythonExpression([
        "'/d405/aligned_depth_to_color/image_raw' if '",
        LaunchConfiguration('depth_topic'),
        "'.lower() == 'auto' else '",
        LaunchConfiguration('depth_topic'),
        "'",
    ])
    vision_camera_info_topic = PythonExpression([
        "'/d405/aligned_depth_to_color/camera_info' if '",
        LaunchConfiguration('camera_info_topic'),
        "'.lower() == 'auto' else '",
        LaunchConfiguration('camera_info_topic'),
        "'",
    ])

    full_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mobile_manipulator_moveit_config'),
                'launch',
                'mobile_manipulator_full_moveit.launch.py',
            ])
        ),
        launch_arguments={
            'can_interface': LaunchConfiguration('can_interface'),
            'default_velocity': LaunchConfiguration('default_velocity'),
            'arm_perf_log_every_n_cycles': LaunchConfiguration('arm_perf_log_every_n_cycles'),
            'ros_position_offset_wrist_3_joint': LaunchConfiguration(
                'ros_position_offset_wrist_3_joint'
            ),
            'enable_arm_hardware': LaunchConfiguration('enable_arm_hardware'),
            'port_front': LaunchConfiguration('port_front'),
            'port_rear': LaunchConfiguration('port_rear'),
            'front_driver_id': LaunchConfiguration('front_driver_id'),
            'rear_driver_id': LaunchConfiguration('rear_driver_id'),
            'mobile_baudrate': LaunchConfiguration('mobile_baudrate'),
            'enable_mobile_hardware': LaunchConfiguration('enable_mobile_hardware'),
            'gripper_port': LaunchConfiguration('gripper_port'),
            'gripper_baudrate': LaunchConfiguration('gripper_baudrate'),
            'gripper_id': LaunchConfiguration('gripper_id'),
            'enable_gripper_hardware': LaunchConfiguration('enable_gripper_hardware'),
            'dynamixel_sdk_lib_dir': LaunchConfiguration('dynamixel_sdk_lib_dir'),
            'arm_mount_x': LaunchConfiguration('arm_mount_x'),
            'arm_mount_y': LaunchConfiguration('arm_mount_y'),
            'arm_mount_z': LaunchConfiguration('arm_mount_z'),
            'arm_mount_roll': LaunchConfiguration('arm_mount_roll'),
            'arm_mount_pitch': LaunchConfiguration('arm_mount_pitch'),
            'arm_mount_yaw': LaunchConfiguration('arm_mount_yaw'),
            'gripper_mount_x': LaunchConfiguration('gripper_mount_x'),
            'gripper_mount_y': LaunchConfiguration('gripper_mount_y'),
            'gripper_mount_z': LaunchConfiguration('gripper_mount_z'),
            'gripper_mount_roll': LaunchConfiguration('gripper_mount_roll'),
            'gripper_mount_pitch': LaunchConfiguration('gripper_mount_pitch'),
            'gripper_mount_yaw': LaunchConfiguration('gripper_mount_yaw'),
            'rviz': LaunchConfiguration('rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level'),
            'controllers_file': controllers_file,
            'publish_world_odom_tf': publish_world_odom_tf,
            'publish_static_odom_base_tf': publish_world_odom_tf,
        }.items(),
    )

    nav_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('frjoco_bringup'),
                'launch',
                'internal',
                'sensors_hardware_impl.launch.py',
            ])
        ),
        launch_arguments={
            'enable_rviz': 'false',
            'frame_id': LaunchConfiguration('nav_scan_target_frame'),
        }.items(),
    )

    rtab_livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('frjoco_bringup'),
                'launch',
                'internal',
                'rtab_livox.launch.py',
            ])
        ),
        launch_arguments={
            'localization': LaunchConfiguration('nav_localization'),
            'lidar_topic': '/livox/lidar',
            'frame_id': 'base_footprint',
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('frjoco_bringup'),
                'launch',
                'internal',
                'nav2_hardware_impl.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scan_target_frame': LaunchConfiguration('nav_scan_target_frame'),
        }.items(),
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_multi_camera_launch.py',
            ])
        ),
        launch_arguments={
            'camera_name1': 'd455',
            'camera_namespace1': '',
            'serial_no1': LaunchConfiguration('d455_serial'),
            'rgb_camera.color_profile1': LaunchConfiguration('d455_color_profile'),
            'enable_depth1': 'false',
            'enable_gyro1': 'false',
            'enable_accel1': 'false',
            'align_depth.enable1': 'false',
            'pointcloud.enable1': 'false',
            'camera_name2': 'd405',
            'camera_namespace2': '',
            'serial_no2': LaunchConfiguration('d405_serial'),
            'depth_module.color_profile2': LaunchConfiguration('d405_color_profile'),
            'align_depth.enable2': 'true',
            'pointcloud.enable2': 'false',
        }.items(),
    )

    yolo_seg = Node(
        package='yolo_realsense',
        executable='yolo_seg_node',
        name='yolo26_seg_mask_publisher',
        output='screen',
        additional_env={
            'PYTHONPATH': [
                LaunchConfiguration('yolo_pythonpath'),
                ':',
                EnvironmentVariable('PYTHONPATH', default_value=''),
            ],
        },
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'color_topic': vision_color_topic,
            'confidence_threshold': ParameterValue(
                LaunchConfiguration('confidence_threshold'),
                value_type=float,
            ),
            'target_class_id': ParameterValue(
                LaunchConfiguration('target_class_id'),
                value_type=int,
            ),
            'imgsz': ParameterValue(
                LaunchConfiguration('yolo_imgsz'),
                value_type=int,
            ),
            'retina_masks': ParameterValue(
                LaunchConfiguration('yolo_retina_masks'),
                value_type=bool,
            ),
            'max_inference_hz': ParameterValue(
                LaunchConfiguration('yolo_max_inference_hz'),
                value_type=float,
            ),
            'preview_publish_hz': ParameterValue(
                LaunchConfiguration('yolo_preview_publish_hz'),
                value_type=float,
            ),
            'show_visualization': ParameterValue(
                LaunchConfiguration('show_yolo_visualization'),
                value_type=bool,
            ),
        }],
    )

    thin_part_grasp = Node(
        package='yolo_realsense',
        executable='thin_part_grasp_node',
        name='thin_part_grasp_trigger_node',
        output='screen',
        parameters=[{
            'mask_topic': '/yolo_seg/target_mask',
            'target_info_topic': '/yolo_seg/target_info',
            'color_topic': vision_color_topic,
            'depth_topic': vision_depth_topic,
            'camera_info_topic': vision_camera_info_topic,
            'camera_frame': LaunchConfiguration('camera_frame'),
            'enable_keyboard_trigger': ParameterValue(
                LaunchConfiguration('enable_keyboard_trigger'),
                value_type=bool,
            ),
            'show_visualization': ParameterValue(
                LaunchConfiguration('show_yolo_visualization'),
                value_type=bool,
            ),
        }],
    )

    yolo_bridge = Node(
        package='mobile_manipulator_moveit_config',
        executable='yolo_grasp_pose_bridge.py',
        name='yolo_grasp_pose_bridge',
        output='screen',
        parameters=[{
            'output_topic': '/target_pose',
            'execute_service': '/execute_pick_place',
            'camera_frame': LaunchConfiguration('camera_frame'),
            'target_object': 'latest',
            'require_roll': True,
            'roll_axis': 'z',
            'auto_publish': False,
        }],
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
            'require_confirmation': ParameterValue(
                LaunchConfiguration('require_confirmation'),
                value_type=bool,
            ),
            'home_on_start': ParameterValue(
                LaunchConfiguration('home_on_start'),
                value_type=bool,
            ),
            'run_grasp_sequence': ParameterValue(
                LaunchConfiguration('enable_gripper_hardware'),
                value_type=bool,
            ),
            'status_topic': '/pick_place_status',
            'open_before_grasp': True,
            'force_open_before_close': True,
            'hold_manual_gripper_open': ParameterValue(
                LaunchConfiguration('hold_manual_gripper_open'),
                value_type=bool,
            ),
            'gripper_hold_joint_name': LaunchConfiguration('gripper_hold_joint_name'),
            'gripper_open_position': 0.029,
            'gripper_close_position': 0.000,
            'gripper_max_effort': 50.0,
            'grasp_roll_enabled': ParameterValue(
                LaunchConfiguration('grasp_roll_enabled'),
                value_type=bool,
            ),
            'grasp_info_topic': '/thin_part/grasp_info',
            'grasp_roll_joint_name': LaunchConfiguration('grasp_roll_joint_name'),
            'grasp_roll_sign': ParameterValue(
                LaunchConfiguration('grasp_roll_sign'),
                value_type=float,
            ),
            'grasp_roll_offset_deg': ParameterValue(
                LaunchConfiguration('grasp_roll_offset_deg'),
                value_type=float,
            ),
            'grasp_roll_max_abs_deg': ParameterValue(
                LaunchConfiguration('grasp_roll_max_abs_deg'),
                value_type=float,
            ),
            'grasp_roll_max_age_sec': ParameterValue(
                LaunchConfiguration('grasp_roll_max_age_sec'),
                value_type=float,
            ),
            'place_transition_mode': LaunchConfiguration('place_transition_mode'),
            'load_turn_joint_name': LaunchConfiguration('load_turn_joint_name'),
            'load_turn_angle': ParameterValue(
                LaunchConfiguration('load_turn_angle'),
                value_type=float,
            ),
            'carry_elbow_extra_bend_deg': ParameterValue(
                LaunchConfiguration('carry_elbow_extra_bend_deg'),
                value_type=float,
            ),
            'pre_grasp_align_enabled': ParameterValue(
                LaunchConfiguration('pre_grasp_align_enabled'),
                value_type=bool,
            ),
            'pre_grasp_align_x_offset': ParameterValue(
                LaunchConfiguration('pre_grasp_align_x_offset'),
                value_type=float,
            ),
            'pre_grasp_align_settle_sec': ParameterValue(
                LaunchConfiguration('pre_grasp_align_settle_sec'),
                value_type=float,
            ),
            'post_place_turn_enabled': ParameterValue(
                LaunchConfiguration('post_place_turn_enabled'),
                value_type=bool,
            ),
            'post_place_turn_joint_name': LaunchConfiguration('post_place_turn_joint_name'),
            'post_place_turn_angle': ParameterValue(
                LaunchConfiguration('post_place_turn_angle'),
                value_type=float,
            ),
            'pre_open_after_place_delay_sec': ParameterValue(
                LaunchConfiguration('pre_open_after_place_delay_sec'),
                value_type=float,
            ),
            'place_shoulder_pan_joint': ParameterValue(
                LaunchConfiguration('place_shoulder_pan'),
                value_type=float,
            ),
            'place_shoulder_lift_joint': ParameterValue(
                LaunchConfiguration('place_shoulder_lift'),
                value_type=float,
            ),
            'place_elbow_joint': ParameterValue(
                LaunchConfiguration('place_elbow'),
                value_type=float,
            ),
            'place_wrist_1_joint': ParameterValue(
                LaunchConfiguration('place_wrist_1'),
                value_type=float,
            ),
            'place_wrist_2_joint': ParameterValue(
                LaunchConfiguration('place_wrist_2'),
                value_type=float,
            ),
            'place_wrist_3_joint': ParameterValue(
                LaunchConfiguration('place_wrist_3'),
                value_type=float,
            ),
            'open_after_place': True,
            'return_ready_after_place': True,
            'gripper_settle_sec': 1.0,
            'post_grasp_settle_sec': 0.5,
        }],
    )

    web_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_web_interface'),
                'launch',
                'web_interface.launch.py',
            ])
        ),
    )

    hardware_status = Node(
        package='frjoco_bringup',
        executable='hardware_status_monitor.py',
        name='hardware_status_monitor',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'color_topic': vision_color_topic,
            'depth_topic': vision_depth_topic,
            'camera_info_topic': vision_camera_info_topic,
        }],
    )

    mobile_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='mobile_keyboard_teleop',
        output='screen',
        prefix='xterm -e',
        remappings=[
            ('/cmd_vel', LaunchConfiguration('mobile_teleop_topic')),
        ],
        parameters=[{
            'speed': ParameterValue(
                LaunchConfiguration('mobile_teleop_speed'),
                value_type=float,
            ),
            'turn': ParameterValue(
                LaunchConfiguration('mobile_teleop_turn'),
                value_type=float,
            ),
        }],
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
        }],
    )

    return LaunchDescription(args + [
        full_moveit,
        TimerAction(
            period=4.0,
            actions=[ee_pose_publisher],
            condition=IfCondition(LaunchConfiguration('enable_ee_pose_publisher')),
        ),
        TimerAction(
            period=2.0,
            actions=[hardware_status],
            condition=IfCondition(LaunchConfiguration('enable_hardware_status')),
        ),
        TimerAction(
            period=8.0,
            actions=[mobile_teleop],
            condition=IfCondition(LaunchConfiguration('enable_mobile_teleop')),
        ),
        TimerAction(
            period=3.0,
            actions=[realsense],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('enable_realsense'), "'.lower() == 'true' and '",
                LaunchConfiguration('enable_nav_sensors'), "'.lower() != 'true'",
            ])),
        ),
        TimerAction(
            period=6.0,
            actions=[yolo_seg, thin_part_grasp],
            condition=IfCondition(LaunchConfiguration('enable_yolo')),
        ),
        TimerAction(
            period=11.0,
            actions=[yolo_bridge, vision_move],
            condition=IfCondition(LaunchConfiguration('enable_move_executor')),
        ),
        TimerAction(
            period=13.0,
            actions=[web_interface],
            condition=IfCondition(LaunchConfiguration('enable_web')),
        ),
        TimerAction(
            period=3.0,
            actions=[nav_sensors],
            condition=IfCondition(LaunchConfiguration('enable_nav_sensors')),
        ),
        TimerAction(
            period=9.0,
            actions=[rtab_livox],
            condition=IfCondition(LaunchConfiguration('enable_rtab_livox')),
        ),
        TimerAction(
            period=15.0,
            actions=[nav2],
            condition=IfCondition(LaunchConfiguration('enable_nav2')),
        ),
    ])
