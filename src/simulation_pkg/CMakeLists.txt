cmake_minimum_required(VERSION 3.8)
project(simulation_pkg)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds maps launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
