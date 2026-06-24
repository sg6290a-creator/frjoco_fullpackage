cmake_minimum_required(VERSION 3.16)
project(frjoco_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/topic_pick_place.py
    scripts/hardware_status_monitor.py
    scripts/can_motor_hz_monitor.py
    scripts/cmd_vel_capper.py
    scripts/obstacle_decay_node.py
    scripts/sim_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()
