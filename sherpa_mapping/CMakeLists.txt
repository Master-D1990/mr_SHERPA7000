cmake_minimum_required(VERSION 3.0.2)
project(sherpa_mapping)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  nav_msgs
  sensor_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs tf geometry_msgs nav_msgs sensor_msgs
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Mark config files for installation
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.config"
)

## Mark rviz config files for installation
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

## Mark maps directory for installation
install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
  PATTERN ".svn" EXCLUDE
)