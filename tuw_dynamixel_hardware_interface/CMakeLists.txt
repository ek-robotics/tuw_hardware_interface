cmake_minimum_required(VERSION 3.0.2)
project(tuw_dynamixel_hardware_interface)

add_compile_options(-std=c++14)

find_package(
  catkin
  REQUIRED
  roscpp
  rospy
  tuw_ros_control_generic
)

catkin_package(
  INCLUDE_DIRS
  include
#  LIBRARIES tuw_dynamixel_hardware_interface
  CATKIN_DEPENDS
  roscpp
  rospy
  tuw_ros_control_generic
#  DEPENDS system_lib
)

include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()
