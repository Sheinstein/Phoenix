cmake_minimum_required(VERSION 2.8.3)
project(phoenix_rescuer)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  ar_track_alvar
  depthimage_to_laserscan
  diagnostic_msgs
  find_object_2d
  geometry_msgs
  gmapping
  image_transport
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
)


################################################
## Declare ROS messages, services and actions ##
################################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES phoenix_rescuer
#  CATKIN_DEPENDS actionlib actionlib ar_track_alvar depthimage_to_laserscan diagnostic_msgs find_object_2d frontier_exploration geometry_msgs gmapping grid_map image_transport move_base nav_msgs rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

