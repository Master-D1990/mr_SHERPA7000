cmake_minimum_required(VERSION 3.0.2)
project(sherpa_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Installationsregeln
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)