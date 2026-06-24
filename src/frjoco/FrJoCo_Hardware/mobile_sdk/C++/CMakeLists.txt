cmake_minimum_required(VERSION 3.14)
project(mobile_sdk_cpp CXX)

# ---------------------------------------------------------------------------
# mobile_sdk native layer — NO ROS dependencies.
#
# 3-layer structure:
#   serial_port  — POSIX serial wrapper
#   md_protocol  — MD200T packet build/parse, structs, constants
#   md_driver    — high-level 4WD controller (uses serial_port + md_protocol)
#
# ros2/ 계층은 MDDriver 를 has-a 멤버로 사용하는 SystemInterface 를 빌드한다.
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# ---------------------------------------------------------------------------
# mobile_sdk_native — 순수 C++ 라이브러리 (ROS 의존 없음)
# ---------------------------------------------------------------------------
add_library(mobile_sdk_native STATIC
  src/serial_port.cpp
  src/md_protocol.cpp
  src/md_driver.cpp
)
set_target_properties(mobile_sdk_native PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(mobile_sdk_native PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(mobile_sdk_native PUBLIC cxx_std_17)

# Link pthread (for std::mutex)
find_package(Threads REQUIRED)
target_link_libraries(mobile_sdk_native PUBLIC Threads::Threads)

message(STATUS "[mobile_sdk/C++] mobile_sdk_native built (pure POSIX, no ROS).")

# ---------------------------------------------------------------------------
# Optional test executable (standalone, no ROS needed)
# ---------------------------------------------------------------------------
option(MOBILE_SDK_BUILD_TESTS "Build native test executables" OFF)

if(MOBILE_SDK_BUILD_TESTS)
  add_executable(test_wheels test/test_wheels.cpp)
  target_link_libraries(test_wheels PRIVATE mobile_sdk_native)
  message(STATUS "[mobile_sdk/C++] test_wheels built.")

  add_executable(test_teleop test/test_teleop.cpp)
  target_link_libraries(test_teleop PRIVATE mobile_sdk_native)
  message(STATUS "[mobile_sdk/C++] test_teleop built.")
endif()
