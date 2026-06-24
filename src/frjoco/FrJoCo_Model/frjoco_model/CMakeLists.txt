cmake_minimum_required(VERSION 3.8)
project(frjoco_model)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  meshes
  URDF
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
