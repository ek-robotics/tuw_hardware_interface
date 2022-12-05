// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_CONFIG_DESCRIPTION_H
#define TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_CONFIG_DESCRIPTION_H

#include <map>

#include <yaml-cpp/yaml.h>

namespace tuw_ros_control_generic
{
class GenericConfigDescription
{
public:
  explicit GenericConfigDescription(YAML::Node yaml);
  std::map<std::string, int> getConfigMap();
private:
  std::map<std::string, int> config_map_;
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_CONFIG_DESCRIPTION_H
