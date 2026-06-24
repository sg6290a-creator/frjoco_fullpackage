cmake_minimum_required(VERSION 3.16)
project(mobile_sdk LANGUAGES CXX)

# ---------------------------------------------------------------------------
# mobile_sdk ROS2 어댑터 계층
#
# 구조:
#   C++/ → mobile_sdk_native (pure C++: SerialPort, MDProtocol, MDDriver)
#   ros2/ → MobileSystemInterface (MDDriver 를 has-a 멤버로 사용)
#
# C++/폴더는 COLCON_IGNORE 로 독립 빌드 가능, 여기서도 소스를 가져와 컴파일.
# ---------------------------------------------------------------------------

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# mobile_sdk_native: SerialPort + MDProtocol + MDDriver (pure C++, no ROS)
# ---------------------------------------------------------------------------
add_library(mobile_sdk_native STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../C++/src/serial_port.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../C++/src/md_protocol.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../C++/src/md_driver.cpp
)
set_target_properties(mobile_sdk_native PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(mobile_sdk_native PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../C++/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(mobile_sdk_native PUBLIC Threads::Threads)
target_compile_features(mobile_sdk_native PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# mobile_sdk_hw: MobileSystemInterface (thin ros2_control bridge)
# ---------------------------------------------------------------------------
add_library(mobile_sdk_hw SHARED
  src/mobile_system_interface.cpp
)

target_include_directories(mobile_sdk_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../C++/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(mobile_sdk_hw PUBLIC mobile_sdk_native)

target_compile_features(mobile_sdk_hw PUBLIC cxx_std_17)

ament_target_dependencies(mobile_sdk_hw PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface mobile_sdk_plugin.xml)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

# Headers
install(
  DIRECTORY ../C++/include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Libraries
install(
  TARGETS mobile_sdk_native mobile_sdk_hw
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugin XML
install(
  FILES mobile_sdk_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
