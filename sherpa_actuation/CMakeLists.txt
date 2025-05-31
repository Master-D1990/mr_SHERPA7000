cmake_minimum_required(VERSION 3.0.2)
project(sherpa_actuation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  sherpa_control
)

## Generate messages if needed
# add_message_files(
#   FILES
#   MotorStatus.msg
# )

## Generate services if needed
# add_service_files(
#   FILES
#   SetMotorSpeed.srv
# )

## Generate added messages and services
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sherpa_actuation
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs message_runtime sherpa_control
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library for the PCA9685 controller
add_library(pca9685
  src/pca9685.cpp
)
target_link_libraries(pca9685 ${catkin_LIBRARIES})

## Declare a C++ library for the TB6612FNG motor driver
add_library(tb6612fng
  src/tb6612fng.cpp
)
target_link_libraries(tb6612fng ${catkin_LIBRARIES} pca9685)

## Declare a C++ library for GPIO control
add_library(gpio_controller
  src/gpio_controller.cpp
)
target_link_libraries(gpio_controller ${catkin_LIBRARIES})

## Declare a C++ library for the motor controller
add_library(motor_controller
  src/motor_controller.cpp
)
target_link_libraries(motor_controller ${catkin_LIBRARIES} tb6612fng pca9685 gpio_controller)

## Declare the main motor node
add_executable(motor_node src/motor_node.cpp)
target_link_libraries(motor_node ${catkin_LIBRARIES} motor_controller)
add_dependencies(motor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install targets
install(TARGETS motor_node pca9685 tb6612fng gpio_controller motor_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)