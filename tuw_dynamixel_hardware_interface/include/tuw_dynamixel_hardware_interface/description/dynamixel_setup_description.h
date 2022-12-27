// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_DYNAMIXEL_SETUP_DESCRIPTION_H
#define DIP_WS_DYNAMIXEL_SETUP_DESCRIPTION_H

#include "tuw_ros_control_generic/description/generic_setup_description.h"
#include "dynamixel_joint_description.h"

using tuw_ros_control_generic::GenericSetupDescription;

namespace tuw_hardware_interface
{
class DynamixelSetupDescription : public GenericSetupDescription
{
public:
  explicit DynamixelSetupDescription(YAML::Node yaml);
  std::list<DynamixelJointDescription> getDynamixelJoints();
protected:
  std::list<DynamixelJointDescription> dynamixel_joints_ {std::list<DynamixelJointDescription>()};
};
}

#endif //DIP_WS_DYNAMIXEL_SETUP_DESCRIPTION_H
