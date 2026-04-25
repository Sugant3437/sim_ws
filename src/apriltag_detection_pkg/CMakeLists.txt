cmake_minimum_required(VERSION 3.8)
project(apriltag_detection_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  apriltag_detection_pkg/apriltag_localizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME apriltag_localizer
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
