cmake_minimum_required(VERSION 3.0.2)
project(tuw_ros_control_generic)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

link_libraries(yaml-cpp)

find_package(
  catkin
  REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  roslint
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  rospy
)


include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_library(
  tuw_ros_control_generic
  src/${PROJECT_NAME}/description/generic_config_description.cpp
  src/${PROJECT_NAME}/description/generic_connection_description.cpp
  src/${PROJECT_NAME}/description/generic_hardware_description.cpp
  src/${PROJECT_NAME}/description/generic_hardware_parameter_description.cpp
  src/${PROJECT_NAME}/description/generic_joint_description.cpp
  src/${PROJECT_NAME}/generic_hardware.cpp
  src/${PROJECT_NAME}/generic_hardware_parameter.cpp
  src/${PROJECT_NAME}/generic_setup_prefix.cpp
)

target_link_libraries(
  tuw_ros_control_generic
  ${catkin_LIBRARIES}
)

# add gtests
catkin_add_gtest(
  tuw_ros_control_generic_units_tests
  # test util
  test/${PROJECT_NAME}_test_util/src/file_loader.cpp
  # test
  test/${PROJECT_NAME}_test/description/test_generic_config_description.cpp
  test/${PROJECT_NAME}_test/description/test_generic_connection_description.cpp
  test/${PROJECT_NAME}_test/description/test_generic_hardware_description.cpp
  test/${PROJECT_NAME}_test/description/test_generic_hardware_parameter_description.cpp
  test/${PROJECT_NAME}_test/description/test_generic_joint_description.cpp
  test/${PROJECT_NAME}_test/test_all.cpp
  test/${PROJECT_NAME}_test/test_generic_hardware.cpp
  test/${PROJECT_NAME}_test/test_generic_hardware_parameter.cpp
  test/${PROJECT_NAME}_test/test_generic_setup_name.cpp
)

# link libraries to gtests
target_link_libraries(
  tuw_ros_control_generic_units_tests
  tuw_ros_control_generic
  # gmock is only reqired if the mocks are created
  # gmock
)