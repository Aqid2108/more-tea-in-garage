cmake_minimum_required(VERSION 3.8)
project(more_tea_base_description)

find_package(ament_cmake REQUIRED)

# meshes/raw (Fusion's millimetre export) is committed for tools/generate_urdf.py but not installed
install(DIRECTORY urdf launch rviz docs DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes/visual DESTINATION share/${PROJECT_NAME}/meshes)

ament_package()
