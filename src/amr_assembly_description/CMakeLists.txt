cmake_minimum_required(VERSION 3.8)
project(amr_assembly_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch rviz config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/dummy_joint_state_publisher.py
  scripts/esp32_driver_node.py
  scripts/esp32_wifi_driver_node.py
  scripts/dummy_laser_scan_publisher.py
  scripts/twist_to_wheel_cmd.py
  scripts/sim_robot_odometry.py
  scripts/slam_lifecycle_auto_starter.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
