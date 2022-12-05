// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_GENERIC_HARDWARE_PARAMETER_DESCRIPTION_H
#define DIP_WS_GENERIC_HARDWARE_PARAMETER_DESCRIPTION_H

#include <map>
#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

namespace tuw_ros_control_generic
{
class GenericHardwareParameterDescription {
public:
  explicit GenericHardwareParameterDescription(YAML::Node yaml);
  std::string* getIdentifier();
  std::string* getDescription();
  int* getAddress();
  int* getLength();
  int* getRangeMax();
  int* getRangeMin();
  std::map<std::string, int>* getRangeEnumMap();
private:
  std::unique_ptr<std::string> identifier_ {nullptr};
  std::unique_ptr<std::string> description_ {nullptr};
  std::unique_ptr<int> address_ {nullptr};
  std::unique_ptr<int> length_ {nullptr};
  std::unique_ptr<int> range_max_ {nullptr};
  std::unique_ptr<int> range_min_ {nullptr};
  std::unique_ptr<std::map<std::string, int>> range_enum_map_ {nullptr};
};
}
#endif //DIP_WS_GENERIC_HARDWARE_PARAMETER_DESCRIPTION_H
