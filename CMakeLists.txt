cmake_minimum_required(VERSION 3.0.2)
project(ros_image_saver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_saver
#  CATKIN_DEPENDS rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
