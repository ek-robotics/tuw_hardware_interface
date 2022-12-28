// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_HARDWARE_INTERFACE_TEMPLATE_DESCRIPTION_GENERIC_HARDWARE_DESCRIPTION_H
#define TUW_HARDWARE_INTERFACE_TEMPLATE_DESCRIPTION_GENERIC_HARDWARE_DESCRIPTION_H

#include <map>
#include <memory>
#include <string>

#include <boost/unordered_map.hpp>

#include <yaml-cpp/yaml.h>

namespace tuw_hardware_interface
{
class GenericHardwareParameterDescription;

class GenericHardwareDescription
{
public:
  explicit GenericHardwareDescription(YAML::Node yaml);
  std::string getName();
  std::shared_ptr<double> getPositionResolution();
  std::shared_ptr<double> getVelocityResolution();
  std::shared_ptr<double> getEffortResolution();
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> getTargetIdentifierToDescription();
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> getActualIdentifierToDescription();
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>> getConfigIdentifierToDescription();
  std::shared_ptr<std::list<std::string>> getConfigIdentifiers();
private:
  std::string name_;
  std::shared_ptr<double> position_resolution_;
  std::shared_ptr<double> velocity_resolution_;
  std::shared_ptr<double> effort_resolution_;
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>>
          target_identifier_to_description_{nullptr};
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>>
          actual_identifier_to_description_{nullptr};
  std::shared_ptr<std::map<std::string, GenericHardwareParameterDescription>>
          config_identifier_to_description_{nullptr};
  std::shared_ptr<std::list<std::string>> config_identifiers_;
};
}  // namespace tuw_hardware_interface

#endif  // TUW_HARDWARE_INTERFACE_TEMPLATE_DESCRIPTION_GENERIC_HARDWARE_DESCRIPTION_H
