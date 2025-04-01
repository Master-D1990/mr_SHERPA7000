cmake_minimum_required(VERSION 3.0.2)
project(sherpa_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  sherpa_description
  sherpa_control
)

catkin_package(
  CATKIN_DEPENDS gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Installationsregeln
install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)