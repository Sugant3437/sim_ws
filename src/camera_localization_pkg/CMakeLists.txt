cmake_minimum_required(VERSION 3.8)
project(camera_localization_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  camera_localization_pkg/pose_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_relay
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
