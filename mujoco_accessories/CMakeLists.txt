cmake_minimum_required(VERSION 3.10)
project(mujoco_accessories)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  configs
  mujoco
  meshes
  robot
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
