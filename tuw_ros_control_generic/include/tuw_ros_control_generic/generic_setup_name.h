// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_GENERIC_LOGGING_TOOL_H
#define TUW_ROS_CONTROL_GENERIC_GENERIC_LOGGING_TOOL_H

#include <string>
#include <memory>

#define LOG c_str()

namespace tuw_ros_control_generic
{
class GenericSetupName
{
public:
  GenericSetupName() = default;
  ~GenericSetupName() = default;
  static void setSetupName(const std::string& setup_name);
  static std::string getSetupName();
protected:
  static inline std::string setup_name_;
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_GENERIC_LOGGING_TOOL_H
