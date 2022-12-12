// Copyright 2022 Eugen Kaltenegger

#include <tuw_ros_control_generic/description/generic_hardware_description.h>
#include <tuw_ros_control_generic/description/generic_hardware_parameter_description.h>

#include <map>
#include <memory>
#include <string>

using tuw_ros_control_generic::GenericHardwareDescription;
using tuw_ros_control_generic::GenericHardwareParameterDescription;

GenericHardwareDescription::GenericHardwareDescription(YAML::Node yaml)
{
  this->target_identifier_to_description_ = std::make_shared<std::map<std::string, GenericHardwareParameterDescription>>();
  this->actual_identifier_to_description_ = std::make_shared<std::map<std::string, GenericHardwareParameterDescription>>();
  this->config_identifier_to_description_ = std::make_shared<std::map<std::string, GenericHardwareParameterDescription>>();

  YAML::Node target_values_yaml = yaml["target_state_parameters"];
  YAML::Node actual_values_yaml = yaml["actual_state_parameters"];
  YAML::Node config_values_yaml = yaml["config_parameters"];

  for (auto target_value_yaml : target_values_yaml)
  {
    std::string key = target_value_yaml["identifier"].as<std::string>();
    this->target_identifier_to_description_->insert({key, GenericHardwareParameterDescription(target_value_yaml)});
  }

  for (auto actual_value_yaml : actual_values_yaml)
  {
    std::string key = actual_value_yaml["identifier"].as<std::string>();
    this->actual_identifier_to_description_->insert({key, GenericHardwareParameterDescription(actual_value_yaml)});
  }

  for (auto config_value_yaml : config_values_yaml)
  {
    std::string key = config_value_yaml["identifier"].as<std::string>();
    this->config_identifier_to_description_->insert({key, GenericHardwareParameterDescription(config_value_yaml)});
  }
}

std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>>
GenericHardwareDescription::getTargetIdentifierToDescription()
{
  return this->target_identifier_to_description_;
}

std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>>
GenericHardwareDescription::getActualIdentifierToDescription()
{
  return this->actual_identifier_to_description_;
}

std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>>
GenericHardwareDescription::getConfigIdentifierToDescription()
{
  return this->config_identifier_to_description_;
}

