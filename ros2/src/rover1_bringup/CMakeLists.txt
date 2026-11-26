cmake_minimum_required(VERSION 3.16)
project(rover1_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf udev DESTINATION share/${PROJECT_NAME})

ament_package()
