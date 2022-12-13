// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_GENERIC_HARDWARE_H
#define TUW_ROS_CONTROL_GENERIC_GENERIC_HARDWARE_H

#include <tuw_ros_control_generic/description/generic_hardware_description.h>
#include <tuw_ros_control_generic/generic_hardware_parameter.h>

namespace tuw_ros_control_generic
{
class GenericHardware
{
public:
  explicit GenericHardware(GenericHardwareDescription hardware_description);
  enum Mode
  {
    POSITION,
    VELOCITY,
    EFFORT
  };
  bool supportsTargetMode(Mode mode);
  bool supportsActualMode(Mode mode);
  GenericHardwareParameter getTargetParameterForMode(Mode mode);
  GenericHardwareParameter getActualParameterForMode(Mode mode);
  std::shared_ptr<std::list<std::string>> getConfigIdentifiers();
  std::shared_ptr<std::map<std::string, GenericHardwareParameter>> getConfigIdentifierToParameter();
private:
  std::map<Mode, GenericHardwareParameter> target_modes_to_parameter_;
  std::map<Mode, GenericHardwareParameter> actual_modes_to_parameter_;
  std::shared_ptr<std::list<std::string>> config_identifiers_;
  std::shared_ptr<std::map<std::string, GenericHardwareParameter>> config_identifier_to_parameter_;
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_GENERIC_HARDWARE_H
