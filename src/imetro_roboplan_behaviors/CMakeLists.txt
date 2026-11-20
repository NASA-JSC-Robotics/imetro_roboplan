cmake_minimum_required(VERSION 3.22)
project(imetro_roboplan_behaviors)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY
    trees
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
