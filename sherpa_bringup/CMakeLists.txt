cmake_minimum_required(VERSION 3.0.2)
project(sherpa_bringup)

find_package(catkin REQUIRED COMPONENTS
  sherpa_description
  sherpa_control
  sherpa_gazebo
)

catkin_package(
  CATKIN_DEPENDS 
    sherpa_description
    sherpa_control
    sherpa_gazebo
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Installationsregeln für Launch-Dateien
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)