cmake_minimum_required(VERSION 3.14)
project(ridgeback_mujoco_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

find_package(controller_manager REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)

# ✅ Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ✅ Install config (YAML) files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ✅ Install URDF/Xacro files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# ✅ Install MuJoCo XML models
install(DIRECTORY mujoco_models
  DESTINATION share/${PROJECT_NAME}
)

# ✅ Install meshes
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
