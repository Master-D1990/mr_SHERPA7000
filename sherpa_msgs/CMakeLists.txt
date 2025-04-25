cmake_minimum_required(VERSION 3.0.2)
project(sherpa_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

# Falls du Nachrichten definieren willst
# add_message_files(
#   FILES
#   YourMessage.msg
# )

# Falls du Services definieren willst
# add_service_files(
#   FILES
#   YourService.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)