cmake_minimum_required(VERSION 3.0.2)
project(sigverse_hsr_nav)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  amcl
  base_local_planner
  map_server
  move_base
  move_base_msgs
  navfn
  voxel_grid
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sigverse_hsr_nav
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)