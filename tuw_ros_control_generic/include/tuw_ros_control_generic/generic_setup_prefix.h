// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_GENERIC_LOGGING_TOOL_H
#define TUW_ROS_CONTROL_GENERIC_GENERIC_LOGGING_TOOL_H

#include <string>
#include <memory>

#include <ros/ros.h>

#define LOG c_str()
#define PREFIX tuw_ros_control_generic::GenericSetupPrefix::getLogPrefix().LOG

namespace tuw_ros_control_generic
{
class GenericSetupPrefix
{
public:
  GenericSetupPrefix() = default;
  ~GenericSetupPrefix() = default;
  static void setSetupName(const std::string& setup_name);
  static std::string getSetupName();
  static std::string getNodeName();
  static std::string getNodeNameLower();
  static std::string getNodeNameUpper();
  static std::string getLogPrefix();
protected:
  static std::string node_name_;
  static std::string setup_name_;
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_GENERIC_LOGGING_TOOL_H
