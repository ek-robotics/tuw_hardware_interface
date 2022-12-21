// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_FILE_LOADER_H
#define DIP_WS_FILE_LOADER_H

#include <fstream>
#include <ros/package.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

namespace tuw_ros_control_generic_test
{
class FileLoader
{
public:
  static YAML::Node loadYAMLFromFile(const std::string& relative_path);
  static std::string loadURDFFromFile(const std::string& relative_path);
private:
};
}  // namespace tuw_ros_control_generic_test

#endif  // DIP_WS_FILE_LOADER_H
