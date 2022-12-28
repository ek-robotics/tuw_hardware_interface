// Copyright 2022 Eugen Kaltenegger

#include <tuw_hardware_interface_dynamixel/description/dynamixel_joint_description.h>
#include <tuw_hardware_interface_dynamixel/description/dynamixel_setup_description.h>

using tuw_hardware_interface::DynamixelSetupDescription;
using tuw_hardware_interface::DynamixelJointDescription;

DynamixelSetupDescription::DynamixelSetupDescription(YAML::Node yaml) : GenericSetupDescription(yaml)
{
  for (const auto& joint_yaml : yaml["joints"])
  {
    DynamixelJointDescription joint_description = DynamixelJointDescription(joint_yaml);
    this->dynamixel_joints_.emplace_back(joint_description);
  }
}

std::list<DynamixelJointDescription> DynamixelSetupDescription::getDynamixelJoints()
{
  return this->dynamixel_joints_;
}
