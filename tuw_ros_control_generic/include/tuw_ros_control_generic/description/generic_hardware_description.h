// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_HARDWARE_DESCRIPTION_H
#define TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_HARDWARE_DESCRIPTION_H

#include <map>
#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

namespace tuw_ros_control_generic
{
class GenericHardwareParameterDescription;
class GenericHardwareDescription
{
public:
  explicit GenericHardwareDescription(YAML::Node yaml);
  std::string getName();
  std::shared_ptr<double> getPositionResolution();
  std::shared_ptr<double> getVelocityResolution();
  std::shared_ptr<double> getEffortResolution();
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> getTargetIdentifierToDescription();
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> getActualIdentifierToDescription();
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> getConfigIdentifierToDescription();
private:
  std::string name_;
  std::shared_ptr<double> position_resolution_;
  std::shared_ptr<double> velocity_resolution_;
  std::shared_ptr<double> effort_resolution_;
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> target_identifier_to_description_ {nullptr};
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> actual_identifier_to_description_ {nullptr};
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> config_identifier_to_description_ {nullptr};
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_HARDWARE_DESCRIPTION_H
