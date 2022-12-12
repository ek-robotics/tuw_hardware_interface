// Copyright 2022 Eugen Kaltenegger

#include "../include/file_loader.h"

YAML::Node tuw_ros_control_generic_test::FileLoader::loadYAMLFromFile(const std::string& relative_path)
{
  {
    std::string absolute_package_path = ros::package::getPath("tuw_ros_control_generic");
    std::string absolute_file_path = absolute_package_path + relative_path;
    return YAML::LoadFile(absolute_file_path);
  }
}
