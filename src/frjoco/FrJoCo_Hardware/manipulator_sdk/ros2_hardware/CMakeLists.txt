cmake_minimum_required(VERSION 3.8)
project(frlab_manipulator_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ---------------------------------------------------------------------------
# Build frlab_manipulator_sdk as a sub-project (plain CMake, no ROS2 deps)
# ---------------------------------------------------------------------------
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../C++
  ${CMAKE_CURRENT_BINARY_DIR}/manipulator_sdk_native
)

# ---------------------------------------------------------------------------
# Hardware interface shared library
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/frlab_manipulator_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(${PROJECT_NAME}
  manipulator_sdk_native
)

pluginlib_export_plugin_description_file(hardware_interface frlab_manipulator_hardware_plugins.xml)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  FILES frlab_manipulator_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
