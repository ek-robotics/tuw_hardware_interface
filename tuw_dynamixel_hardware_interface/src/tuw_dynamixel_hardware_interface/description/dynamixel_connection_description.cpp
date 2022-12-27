// Copyright 2022 Eugen Kaltenegger

#include "../../../include/tuw_dynamixel_hardware_interface/description/dynamixel_connection_description.h"

using tuw_hardware_interface::DynamixelConnectionDescription;

DynamixelConnectionDescription::DynamixelConnectionDescription(YAML::Node yaml) : GenericConnectionDescription(yaml)
{
  this->protocol_ = yaml["protocol"].as<std::string>();
}

std::string DynamixelConnectionDescription::getProtocol() const
{
  return this->protocol_;
}
