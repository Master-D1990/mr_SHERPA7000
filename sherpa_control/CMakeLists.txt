cmake_minimum_required(VERSION 3.0.2)
project(sherpa_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS controller_manager roscpp std_msgs geometry_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Installationsregeln
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)