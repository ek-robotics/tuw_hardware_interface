cmake_minimum_required(VERSION 3.0.2)
project(tuw_dynamixel_hardware_interface)

add_compile_options(-std=c++14)

find_package(
  catkin
  REQUIRED
  roscpp
  rospy
  tuw_ros_control_generic
  controller_manager
  dynamixel_sdk
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES tuw_dynamixel_hardware_interface
  CATKIN_DEPENDS
    roscpp
    rospy
    tuw_ros_control_generic
    controller_manager
    dynamixel_sdk
  #  DEPENDS system_lib
)

include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_library(
  tuw_dynamixel_hardware_interface
  src/${PROJECT_NAME}/description/dynamixel_connection_description.cpp
  src/${PROJECT_NAME}/description/dynamixel_joint_description.cpp
  src/${PROJECT_NAME}/description/dynamixel_setup_description.cpp
  src/${PROJECT_NAME}/dynamixel_connection.cpp
  src/${PROJECT_NAME}/dynamixel_hardware_interface.cpp
)

target_link_libraries(
  tuw_dynamixel_hardware_interface
  ${catkin_LIBRARIES}
)

add_executable(
  tuw_dynamixel_hardware_interface_node
  src/tuw_dynamixel_hardware_interface_node.cpp
)

add_dependencies(
  tuw_dynamixel_hardware_interface_node
  tuw_dynamixel_hardware_interface
)

target_link_libraries(
  tuw_dynamixel_hardware_interface_node
  tuw_dynamixel_hardware_interface
  ${catkin_LIBRARIES}
)