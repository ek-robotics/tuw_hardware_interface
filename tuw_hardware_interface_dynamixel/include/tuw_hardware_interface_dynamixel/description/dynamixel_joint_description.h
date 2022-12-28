// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_HARDWARE_INTERFACE_DYNAMIXEL_DESCRIPTION_DYNAMIXEL_JOINT_DESCRIPTION_H
#define TUW_HARDWARE_INTERFACE_DYNAMIXEL_DESCRIPTION_DYNAMIXEL_JOINT_DESCRIPTION_H

#include <tuw_ros_control_generic/description/generic_joint_description.h>

#include <memory>

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
}  // namespace tuw_hardware_interface

#endif  // TUW_HARDWARE_INTERFACE_DYNAMIXEL_DESCRIPTION_DYNAMIXEL_JOINT_DESCRIPTION_H
