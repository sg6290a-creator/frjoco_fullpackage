"""
Mobile Manipulator MoveIt Demo Launch File

RViz-only planning demo without ros2_control startup.

Usage:
    ros2 launch mobile_manipulator_moveit_config demo.launch.py
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as f:
            return yaml.safe_load(f)
    except:
        return None


def generate_launch_description():
    # Arguments
    declared_arguments = [
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    rviz_arg = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # Package paths
    moveit_config_pkg = get_package_share_directory("mobile_manipulator_moveit_config")

    # Load URDF from manipulator_description package
    desc_pkg = get_package_share_directory("manipulator_description")
    urdf_path = os.path.join(desc_pkg, "URDF", "manipulator_urdf", "ur5e_arm_only.urdf")
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()
    robot_description = {"robot_description": robot_description_content}

    # Load SRDF
    srdf_path = os.path.join(moveit_config_pkg, "config", "mobile_manipulator.srdf")
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # Load configs
    kinematics_yaml = load_yaml("mobile_manipulator_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("mobile_manipulator_moveit_config", "config/joint_limits.yaml")
    ompl_planning_yaml = load_yaml("mobile_manipulator_moveit_config", "config/ompl_planning.yaml")
    moveit_controllers_yaml = load_yaml("mobile_manipulator_moveit_config", "config/moveit_controllers.yaml")
    sensors_3d_yaml = load_yaml("mobile_manipulator_moveit_config", "config/sensors_3d.yaml")

    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    
    # OMPL planning configuration
    robot_description_planning = {
        "robot_description_planning": {
            **joint_limits_yaml,
            **ompl_planning_yaml
        }
    }

    # MoveIt controller manager
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_controllers_yaml.get("moveit_simple_controller_manager", {}),
        "moveit_controller_manager": moveit_controllers_yaml.get(
            "moveit_controller_manager",
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
    }

    # Planning pipeline - OMPL (Open Manipulator X 방식)
    ompl_planning_pipeline_config = {
        "planning_plugins": ompl_planning_yaml.get("planning_plugins", ["ompl_interface/OMPLPlanner"]),
        "request_adapters": ompl_planning_yaml.get("request_adapters", ""),
        "response_adapters": ompl_planning_yaml.get("response_adapters", ""),
    }

    # Trajectory execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Planning scene monitor
    planning_scene_monitor_params = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Octomap configuration
    octomap_config = {
        "octomap_frame": "arm_base_link",
        "octomap_resolution": 0.05,
        "max_range": 2.0,
    }

    # Sensor configuration for 3D perception - DISABLED
    sensors_config = {}  # 비어있음 = 옥토맵 비활성화

    # ========== NODES ==========

    # Robot State Publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Move Group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            planning_scene_monitor_params,
            moveit_controllers,
            octomap_config,
            sensors_config,
            {"use_sim_time": use_sim_time},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    delay_move_group = TimerAction(period=2.0, actions=[move_group_node])

    # RViz
    rviz_config_file = os.path.join(moveit_config_pkg, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(rviz_arg),
    )

    # Delay RViz
    delay_rviz = TimerAction(period=8.0, actions=[rviz_node])

    return LaunchDescription(
        declared_arguments + [
            robot_state_pub,
            delay_move_group,
            delay_rviz,
        ]
    )
