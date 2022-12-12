// Copyright 2022 Eugen Kaltenegger

#include <tuw_ros_control_generic/description/generic_joint_description.h>
#include <tuw_ros_control_generic/description/generic_setup_description.h>

using tuw_ros_control_generic::GenericJointDescription;
using tuw_ros_control_generic::GenericSetupDescription;

GenericSetupDescription::GenericSetupDescription(YAML::Node yaml)
{
  for (const auto& joint_yaml : yaml["joints"])
  {
    this->joints_.emplace_back(GenericJointDescription(joint_yaml));
  }
}

std::list<GenericJointDescription> GenericSetupDescription::getJoints()
{
  return this->joints_;
}
