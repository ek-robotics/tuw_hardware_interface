// Copyright 2022 Eugen Kaltenegger

#include <tuw_hardware_interface_template/description/generic_config_description.h>

#include <map>
#include <string>

using tuw_hardware_interface::GenericConfigDescription;

GenericConfigDescription::GenericConfigDescription(const YAML::Node& yaml)
{
  for (auto key_value_pair : yaml)
  {
    std::string key = key_value_pair.first.as<std::string>();
    int value = key_value_pair.second.as<int>();
    this->config_map_[key] = value;
  }
}

std::map<std::string, int> GenericConfigDescription::getConfigMap()
{
  return this->config_map_;
}
