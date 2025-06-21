cmake_minimum_required(VERSION 3.22)
project(full_scrab_arm)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
