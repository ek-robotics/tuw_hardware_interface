// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_HARDWARE_INTERFACE_TEMPLATE_DESCRIPTION_GENERIC_JOINT_DESCRIPTION_H
#define TUW_HARDWARE_INTERFACE_TEMPLATE_DESCRIPTION_GENERIC_JOINT_DESCRIPTION_H

#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

namespace tuw_hardware_interface
{
class GenericConnectionDescription;
class GenericHardwareDescription;
class GenericConfigDescription;
class GenericJointDescription
{
public:
  explicit GenericJointDescription(YAML::Node yaml);
  int getId();
  std::string getName();
  virtual std::shared_ptr<GenericConnectionDescription> getConnectionDescription();
  std::shared_ptr<GenericHardwareDescription> getHardwareDescription();
  std::shared_ptr<GenericConfigDescription> getConfigDescription();
protected:
  static YAML::Node loadFile(YAML::Node yaml);
  int id_;
  std::string name_;
  std::shared_ptr<GenericConnectionDescription> connection_description_ {nullptr};
  std::shared_ptr<GenericHardwareDescription> hardware_description_ {nullptr};
  std::shared_ptr<GenericConfigDescription> config_description_ {nullptr};
};
}  // namespace tuw_hardware_interface

#endif  // TUW_HARDWARE_INTERFACE_TEMPLATE_DESCRIPTION_GENERIC_JOINT_DESCRIPTION_H