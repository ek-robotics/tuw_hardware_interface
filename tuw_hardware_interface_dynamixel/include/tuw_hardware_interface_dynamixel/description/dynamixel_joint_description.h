// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_DYNAMIXEL_JOINT_DESCRIPTION_H
#define DIP_WS_DYNAMIXEL_JOINT_DESCRIPTION_H

#include <tuw_ros_control_generic/description/generic_joint_description.h>

using tuw_ros_control_generic::GenericJointDescription;

namespace tuw_hardware_interface
{
class DynamixelConnectionDescription;
class DynamixelJointDescription : public GenericJointDescription
{
public:
  explicit DynamixelJointDescription(const YAML::Node& yaml);
  std::shared_ptr<DynamixelConnectionDescription> getDynamixelConnectionDescription();
private:
  std::shared_ptr<DynamixelConnectionDescription> connection_description_ {nullptr};
};
}

#endif //DIP_WS_DYNAMIXEL_JOINT_DESCRIPTION_H
