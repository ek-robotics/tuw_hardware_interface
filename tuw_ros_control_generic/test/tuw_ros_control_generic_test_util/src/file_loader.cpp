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

std::string tuw_ros_control_generic_test::FileLoader::loadURDFFromFile(const std::string &relative_path)
{
  std::string absolute_package_path = ros::package::getPath("tuw_ros_control_generic");
  std::string absolute_file_path = absolute_package_path + relative_path;

  std::ifstream input_file(absolute_file_path);
  if (input_file.is_open())
    return std::string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());
  else
    throw std::runtime_error("can not open file: " + absolute_file_path);
}
