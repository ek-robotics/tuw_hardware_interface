// Copyright 2022 Eugen Kaltenegger

#include "tuw_ros_control_generic/generic_setup_name.h"

using tuw_ros_control_generic::GenericSetupName;

void GenericSetupName::setSetupName(const std::string& setup_name)
{
  GenericSetupName::setup_name_ = setup_name;
}

std::string tuw_ros_control_generic::GenericSetupName::getSetupName()
{
  return GenericSetupName::setup_name_;
}
