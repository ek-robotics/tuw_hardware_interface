// Copyright 2022 Eugen Kaltenegger

#ifndef FILE_LOADER_H
#define FILE_LOADER_H

#include <fstream>
#include <string>

#include <ros/package.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

namespace tuw_hardware_interface
{
class FileLoader
{
public:
  static YAML::Node loadYAMLFromFile(const std::string& relative_path);
  static std::string loadURDFFromFile(const std::string& relative_path);
private:
};
}  // namespace tuw_hardware_interface

#endif  // FILE_LOADER_H
