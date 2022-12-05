// Copyright 2022 Eugen Kaltenegger

#include <tuw_ros_control_generic/description/generic_joint_description.h>

using tuw_ros_control_generic::GenericJointDescription;

GenericJointDescription::GenericJointDescription(YAML::Node yaml)
{
  this->id_ = yaml["id"].as<int>();
  this->name_ = yaml["name"].as<std::string>();
}

int GenericJointDescription::getId()
{
  return this->id_;
}

std::string GenericJointDescription::getName()
{
  return this->name_;
}

