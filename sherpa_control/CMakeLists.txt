cmake_minimum_required(VERSION 3.0.2)
project(sherpa_control)

find_package(catkin REQUIRED COMPONENTS roscpp)

add_library(gpio_controller
  src/gpio_controller.cpp
)
set_target_properties(gpio_controller PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
)
target_link_libraries(gpio_controller ${catkin_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gpio_controller
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(TARGETS gpio_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
