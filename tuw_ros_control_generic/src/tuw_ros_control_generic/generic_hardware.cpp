// Copyright 2022 Eugen Kaltenegger

#include <tuw_ros_control_generic/generic_hardware.h>

using tuw_ros_control_generic::GenericHardware;
using tuw_ros_control_generic::GenericHardwareParameter;

GenericHardware::GenericHardware(GenericHardwareDescription hardware_description)
{
  this->target_modes_to_parameter_ = std::map<Mode, GenericHardwareParameter>();
  this->actual_modes_to_parameter_ = std::map<Mode, GenericHardwareParameter>();
  this->config_identifier_to_parameter_ = std::make_shared<std::map<std::string, GenericHardwareParameter>>();

  for (const auto& key_value_pair : *hardware_description.getTargetIdentifierToDescription())
  {
    auto key = key_value_pair.first;
    auto value = key_value_pair.second;
    if (key == "target_position")
      this->target_modes_to_parameter_.insert({Mode::POSITION, GenericHardwareParameter(value)});
    if (key == "target_velocity")
      this->target_modes_to_parameter_.insert({Mode::VELOCITY, GenericHardwareParameter(value)});
    if (key == "target_effort")
      this->target_modes_to_parameter_.insert({Mode::EFFORT, GenericHardwareParameter(value)});
  }

  for (const auto& key_value_pair : *hardware_description.getActualIdentifierToDescription())
  {
    auto key = key_value_pair.first;
    auto value = key_value_pair.second;
    if (key == "actual_position")
      this->actual_modes_to_parameter_.insert({Mode::POSITION, GenericHardwareParameter(value)});
    if (key == "actual_velocity")
      this->actual_modes_to_parameter_.insert({Mode::VELOCITY, GenericHardwareParameter(value)});
    if (key == "actual_effort")
      this->actual_modes_to_parameter_.insert({Mode::EFFORT, GenericHardwareParameter(value)});
  }

  for (const auto& key_value_pair : *hardware_description.getConfigIdentifierToDescription())
  {
    auto key = key_value_pair.first;
    auto value = key_value_pair.second;
    this->config_identifier_to_parameter_->insert({key, GenericHardwareParameter(value)});
  }
}

bool GenericHardware::supportsTargetMode(GenericHardware::Mode mode)
{
  if (this->target_modes_to_parameter_.find(mode) == this->target_modes_to_parameter_.end())
    return false;
  else
    return true;
}

bool GenericHardware::supportsActualMode(GenericHardware::Mode mode)
{
  if (this->actual_modes_to_parameter_.find(mode) == this->actual_modes_to_parameter_.end())
    return false;
  else
    return true;
}

GenericHardwareParameter GenericHardware::getTargetParameterForMode(GenericHardware::Mode mode)
{
  return this->target_modes_to_parameter_.at(mode);
}

GenericHardwareParameter GenericHardware::getActualParameterForMode(GenericHardware::Mode mode)
{
  return this->actual_modes_to_parameter_.at(mode);
}

std::shared_ptr<std::map<std::string, GenericHardwareParameter>> GenericHardware::getConfigIdentifierToParameter()
{
  return this->config_identifier_to_parameter_;
}


