cmake_minimum_required(VERSION 3.8)
project(gcs_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
