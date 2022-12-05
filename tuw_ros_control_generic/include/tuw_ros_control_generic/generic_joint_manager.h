// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_GENERIC_JOINT_MANAGER_H
#define DIP_WS_GENERIC_JOINT_MANAGER_H

namespace tuw_ros_control_generic
{
class GenericJointManager {
public:
  GenericJointManager(GenericConnectionDescription connection_description,
                      GenericHardwareDescription hardware_description,
                      GenericConfigDescription config_description);
private:
  std::shared_ptr<GenericConnection> connection_ {nullptr};
  std::shared_ptr<GenericHardware> hardware_ {nullptr};
  std::shared_ptr<GenericConfig> config_ {nullptr};
};
}

#endif //DIP_WS_GENERIC_JOINT_MANAGER_H
