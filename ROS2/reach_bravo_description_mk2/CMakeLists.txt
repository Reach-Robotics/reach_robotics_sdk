cmake_minimum_required(VERSION 3.5)
project(reach_bravo_description_mk2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz urdf DESTINATION share/${PROJECT_NAME})

ament_package()
