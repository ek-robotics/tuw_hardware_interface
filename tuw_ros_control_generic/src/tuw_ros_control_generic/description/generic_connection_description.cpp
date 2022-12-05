// Copyright 2021 Eugen Kaltenegger

#include <tuw_ros_control_generic/description/generic_connection_description.h>

using tuw_ros_control_generic::GenericConnectionDescription;

GenericConnectionDescription::GenericConnectionDescription(YAML::Node yaml)
{
  this->hash_ = yaml["port"].as<std::string>() + "_" + yaml["baudrate"].as<std::string>();
  this->port_ = yaml["port"].as<std::string>();
  this->baudrate_ = yaml["baudrate"].as<int>();
}

std::string GenericConnectionDescription::getHash()
{
  return this->hash_;
}

std::string GenericConnectionDescription::getPort()
{
  return this->port_;
}

int GenericConnectionDescription::getBaudrate()
{
  return this->baudrate_;
}
