cmake_minimum_required(VERSION 3.8)
project(manipulator_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  ../meshes
  ../URDF
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
