// Copyright 2022 Eugen Kaltenegger

#include <tuw_ros_control_generic/description/generic_setup_description.h>

#include <list>
#include <string>

#include <tuw_ros_control_generic/description/generic_joint_description.h>

using tuw_ros_control_generic::GenericJointDescription;
using tuw_ros_control_generic::GenericSetupDescription;

GenericSetupDescription::GenericSetupDescription(YAML::Node yaml)
{
  this->name_ = yaml["setup"].as<std::string>();
  for (const auto& joint_yaml : yaml["joints"])
  {
    this->joints_.emplace_back(GenericJointDescription(joint_yaml));
  }
}

std::string GenericSetupDescription::getName()
{
  return this->name_;
}

std::list<GenericJointDescription> GenericSetupDescription::getJoints()
{
  return this->joints_;
}
