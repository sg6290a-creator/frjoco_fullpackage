cmake_minimum_required(VERSION 3.8)
project(qp_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# OsqpEigen
find_package(OsqpEigen REQUIRED PATHS /usr/local/lib/cmake/OsqpEigen)

# OSQP (from ROS vendor)
find_package(osqp REQUIRED PATHS /opt/ros/humble/lib/cmake/osqp)

# RBDL
find_path(RBDL_INCLUDE_DIR rbdl/rbdl.h PATHS /usr/local/include /usr/include)
find_library(RBDL_LIB NAMES rbdl PATHS /usr/local/lib /usr/lib)
if(NOT RBDL_INCLUDE_DIR OR NOT RBDL_LIB)
  message(FATAL_ERROR "RBDL not found")
endif()

add_library(${PROJECT_NAME} SHARED
  src/qp_solver.cpp
  src/qp_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${RBDL_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  std_msgs
  Eigen3
)

target_link_libraries(${PROJECT_NAME}
  OsqpEigen::OsqpEigen
  osqp::osqp
  ${RBDL_LIB}
)

pluginlib_export_plugin_description_file(controller_interface qp_controller_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES qp_controller_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  controller_interface hardware_interface pluginlib
  rclcpp rclcpp_lifecycle geometry_msgs std_msgs Eigen3
)

ament_package()
