cmake_minimum_required(VERSION 2.8.3)
project(motors)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motors
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

