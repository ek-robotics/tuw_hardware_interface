// Copyright 2022 Eugen Kaltenegger

#include <tuw_hardware_interface_template/description/generic_config_description.h>

#include <map>
#include <string>

using tuw_hardware_interface::GenericConfigDescription;

GenericConfigDescription::GenericConfigDescription(const YAML::Node& yaml)
{
  for (auto key_value_pair : yaml)
  {
    std::pair<std::string, int> pair = {key_value_pair.first.as<std::string>(), key_value_pair.second.as<int>()};
    this->config_.emplace_back(pair);
//    std::string key = key_value_pair.first.as<std::string>();
//    int value = key_value_pair.second.as<int>();
//    this->config_map_[key] = value;
  }
}

std::vector<std::pair<std::string, int>> GenericConfigDescription::getConfig()
{
  return this->config_;
}
