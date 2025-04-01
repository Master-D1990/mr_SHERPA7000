cmake_minimum_required(VERSION 3.0.2)
project(sherpa_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
  robot_state_publisher
  joint_state_publisher
)

catkin_package(
  CATKIN_DEPENDS urdf xacro
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Installationsregeln
install(DIRECTORY urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)