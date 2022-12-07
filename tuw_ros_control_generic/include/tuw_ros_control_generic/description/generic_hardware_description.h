// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_HARDWARE_DESCRIPTION_H
#define TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_HARDWARE_DESCRIPTION_H

#include <map>
#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include <tuw_ros_control_generic/description/generic_hardware_parameter_description.h>

namespace tuw_ros_control_generic
{
class GenericHardwareDescription
{
public:
  explicit GenericHardwareDescription(YAML::Node yaml);
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> getTargetValues();
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> getActualValues();
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> getConfigValues();
private:
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> target_values_ {nullptr};
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> actual_values_ {nullptr};
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> config_values_ {nullptr};
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_HARDWARE_DESCRIPTION_H
