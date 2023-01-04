cmake_minimum_required(VERSION 3.0.2)
project(tuw_hardware_interface_trinamic)

add_compile_options(-std=c++14)

link_libraries(yaml-cpp)

find_package(catkin
  REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  urdf
  tuw_hardware_interface_template
  ddynamic_reconfigure
  controller_manager
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  rospy
  roslint
  urdf
  tuw_hardware_interface_template
  ddynamic_reconfigure
  controller_manager
  #  DEPENDS system_lib
)

include_directories(
 ./include
 ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_library(
  tuw_hardware_interface_trinamic
  src/${PROJECT_NAME}/description/trinamic_hardware_description.cpp
  src/${PROJECT_NAME}/description/trinamic_hardware_parameter_description.cpp
  src/${PROJECT_NAME}/description/trinamic_joint_description.cpp
  src/${PROJECT_NAME}/description/trinamic_setup_description.cpp
  src/${PROJECT_NAME}/trinamic_connection/trinamic_message/trinamic_command.cpp
  src/${PROJECT_NAME}/trinamic_connection/trinamic_message/trinamic_message.cpp
  src/${PROJECT_NAME}/trinamic_connection/trinamic_message/trinamic_reply.cpp
  src/${PROJECT_NAME}/trinamic_connection/tmcm1640_connection.cpp
  src/${PROJECT_NAME}/trinamic_config.cpp
  src/${PROJECT_NAME}/trinamic_hardware.cpp
  src/${PROJECT_NAME}/trinamic_hardware_interface.cpp
  src/${PROJECT_NAME}/trinamic_hardware_parameter.cpp
  src/${PROJECT_NAME}/trinamic_joint.cpp
)

target_link_libraries(
  tuw_hardware_interface_trinamic
  ${catkin_LIBRARIES}
)

add_executable(
  tuw_hardware_interface_trinamic_node
  src/trinamic_hardware_interface_node.cpp
)

add_dependencies(
  tuw_hardware_interface_trinamic_node
  tuw_hardware_interface_trinamic
)

target_link_libraries(
  tuw_hardware_interface_trinamic_node
  tuw_hardware_interface_trinamic
  ${catkin_LIBRARIES}
)
