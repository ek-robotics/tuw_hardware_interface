// Copyright 2022 Eugen Kaltenegger

#include <ros/package.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

YAML::Node loadYAMLFromFile(std::string relative_path)
{
  std::string path = ros::package::getPath("tuw_ros_control_generic") + std::string(relative_path);
  printf("%s", path.c_str());
  return YAML::LoadFile(path);
}
