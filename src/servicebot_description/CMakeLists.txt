cmake_minimum_required(VERSION 3.8)
project(servicebot_description)

find_package(ament_cmake REQUIRED)

# Install package data
install(DIRECTORY urdf meshes launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dummy_joint_state_publisher.py
  scripts/esp32_driver_node.py
  scripts/esp32_wifi_driver_node.py
  scripts/dummy_laser_scan_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
