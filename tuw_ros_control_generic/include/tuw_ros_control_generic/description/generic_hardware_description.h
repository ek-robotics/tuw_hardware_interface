// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_GENERIC_HARDWARE_DESCRIPTION_H
#define DIP_WS_GENERIC_HARDWARE_DESCRIPTION_H

#include <yaml-cpp/yaml.h>

#include <tuw_ros_control_generic/description/generic_hardware_parameter_description.h>

namespace tuw_ros_control_generic
{
class GenericHardwareDescription {
public:
  explicit GenericHardwareDescription(YAML::Node yaml);
  std::map<std::string, GenericHardwareParameterDescription>* getTargetValues();
  std::map<std::string, GenericHardwareParameterDescription>* getActualValues();
  std::map<std::string, GenericHardwareParameterDescription>* getConfigValues();

private:
  std::unique_ptr<std::map<std::string, GenericHardwareParameterDescription>> target_values_ {nullptr};
  std::unique_ptr<std::map<std::string, GenericHardwareParameterDescription>> actual_values_ {nullptr};
  std::unique_ptr<std::map<std::string, GenericHardwareParameterDescription>> config_values_ {nullptr};
};
}

#endif //DIP_WS_GENERIC_HARDWARE_DESCRIPTION_H
