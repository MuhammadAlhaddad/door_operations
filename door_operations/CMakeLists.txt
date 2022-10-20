cmake_minimum_required(VERSION 3.0.2)
project(door_operations)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
  image_transport
  genmsg
  actionlib_msgs
  actionlib
  tf
 )
# Generate actions in the 'action' folder
add_action_files(
  FILES
  Navigate2D.action
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  actionlib_msgs
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(door_operations src/main.cpp)
target_link_libraries(door_operations ${catkin_LIBRARIES})
add_dependencies(door_operations door_operations_generate_messages_cpp)


