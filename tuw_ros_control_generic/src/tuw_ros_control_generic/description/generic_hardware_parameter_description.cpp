// Copyright 2022 Eugen Kaltenegger

#include <tuw_ros_control_generic/description/generic_hardware_parameter_description.h>

#include <map>
#include <memory>
#include <string>
#include <utility>

using tuw_ros_control_generic::GenericHardwareParameterDescription;

GenericHardwareParameterDescription::GenericHardwareParameterDescription(YAML::Node yaml)
{
  // REQUIRED VALUES
  this->identifier_ = std::make_unique<std::string>(yaml["identifier"].as<std::string>());
  this->address_ = std::make_unique<int>(yaml["address"].as<int>());
  this->length_ = std::make_unique<int>(yaml["length"].as<int>());
  // OPTIONAL VALUES
  // optional: description
  try
  {
    this->description_ = std::make_unique<std::string>(yaml["description"].as<std::string>());
  }
  catch (...)
  {
    this->description_ = std::make_unique<std::string>("no description provided");
  }
  // optional: range (max, min)
  try
  {
    this->range_max_ = std::make_unique<int>(yaml["range"]["max"].as<int>());
    this->range_min_ = std::make_unique<int>(yaml["range"]["min"].as<int>());
  }
  catch (...)
  {
    this->range_max_ = nullptr;
    this->range_min_ = nullptr;
  }
  // optional: range (enum)
  try
  {
    YAML::Node range_enum_node = yaml["enum"];

    if (!range_enum_node.IsDefined())
    {
      throw std::runtime_error("no enum");
    }

    this->range_enum_map_ = std::make_unique<std::map<std::string, int>>();

    for (YAML::const_iterator iterator = range_enum_node.begin() ; iterator != range_enum_node.end(); ++iterator)
    {
      std::string key = iterator->first.as<std::string>();
      int value = iterator->second.as<int>();
      this->range_enum_map_->insert(std::pair<std::string, int>(key, value));
    }
  }
  catch (...)
  {
    this->range_enum_map_ = nullptr;
  }
}

std::string* GenericHardwareParameterDescription::getIdentifier()
{
  return this->identifier_.get();
}

std::string* GenericHardwareParameterDescription::getDescription()
{
  return this->description_.get();
}

int* GenericHardwareParameterDescription::getAddress()
{
  return this->address_.get();
}

int* GenericHardwareParameterDescription::getLength()
{
  return this->length_.get();
}

int* GenericHardwareParameterDescription::getRangeMax()
{
  return this->range_max_.get();
}

int* GenericHardwareParameterDescription::getRangeMin()
{
  return this->range_min_.get();
}

std::map<std::string, int>* GenericHardwareParameterDescription::getRangeEnumMap()
{
  return this->range_enum_map_.get();
}
