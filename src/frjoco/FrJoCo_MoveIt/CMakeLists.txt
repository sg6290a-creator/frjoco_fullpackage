cmake_minimum_required(VERSION 3.22)
project(mobile_manipulator_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/vision_move.py
    scripts/terminal_target.py
    scripts/yolo_grasp_pose_bridge.py
    scripts/publish_robot_description.py
    scripts/demo_joint_state_publisher.py
    scripts/passive_joint_state_publisher.py
    scripts/ee_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
