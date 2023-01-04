// Copyright 2023 Eugen Kaltenegger

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tuw_hardware_interface_trinamic");
  ros::NodeHandle basic_nh;
  ros::NodeHandle hardware_nh("/hardware");
  ros::NodeHandle controller_nh("/controller");
  ROS_INFO("HELLO WORLD");
}