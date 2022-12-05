// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_GENERIC_JOINT_MANAGER_H
#define TUW_ROS_CONTROL_GENERIC_GENERIC_JOINT_MANAGER_H

#include <memory>

namespace tuw_ros_control_generic
{
class GenericJointManager
{
public:
  GenericJointManager(GenericConnectionDescription connection_description,
                      GenericHardwareDescription hardware_description,
                      GenericConfigDescription config_description);
private:
  std::shared_ptr<GenericConnection> connection_ {nullptr};
  std::shared_ptr<GenericHardware> hardware_ {nullptr};
  std::shared_ptr<GenericConfig> config_ {nullptr};
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_GENERIC_JOINT_MANAGER_H
