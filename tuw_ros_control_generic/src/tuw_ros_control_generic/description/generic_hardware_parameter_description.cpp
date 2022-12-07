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
  this->identifier_ = std::make_shared<std::string>(yaml["identifier"].as<std::string>());
  this->address_ = std::make_shared<int>(yaml["address"].as<int>());
  this->length_ = std::make_shared<int>(yaml["length"].as<int>());
  // OPTIONAL VALUES
  // optional: description
  try
  {
    this->description_ = std::make_shared<std::string>(yaml["description"].as<std::string>());
  }
  catch (...)
  {
    this->description_ = nullptr;
  }
  // optional: range (enum)
  try
  {
    YAML::Node range_enum_node = yaml["enum"];

    if (!range_enum_node.IsDefined())
    {
      throw std::runtime_error("no enum");
    }

    this->enum_ = std::make_shared<std::map<std::string, int>>();

    for (YAML::const_iterator iterator = range_enum_node.begin() ; iterator != range_enum_node.end(); ++iterator)
    {
      std::string key = iterator->first.as<std::string>();
      int value = iterator->second.as<int>();
      this->enum_->insert(std::pair<std::string, int>(key, value));
    }
  }
  catch (...)
  {
    this->enum_ = nullptr;
  }
  // optional: range (max, min)
  try
  {
    int min = yaml["range"]["min"].as<int>();
    int max = yaml["range"]["max"].as<int>();
    this->range_ = std::make_shared<std::map<std::string, int>>();
    this->range_->insert(std::pair<std::string, int>("min", min));
    this->range_->insert(std::pair<std::string, int>("max", max));
  }
  catch (...)
  {
    this->range_ = nullptr;
  }
}

std::shared_ptr<std::string> GenericHardwareParameterDescription::getIdentifier()
{
  return this->identifier_;
}

std::shared_ptr<std::string> GenericHardwareParameterDescription::getDescription()
{
  return this->description_;
}

std::shared_ptr<int> GenericHardwareParameterDescription::getAddress()
{
  return this->address_;
}

std::shared_ptr<int> GenericHardwareParameterDescription::getLength()
{
  return this->length_;
}

std::shared_ptr<std::map<std::string, int>> GenericHardwareParameterDescription::getEnum()
{
  return this->enum_;
}

std::shared_ptr<std::map<std::string, int>> GenericHardwareParameterDescription::getRange()
{
  return this->range_;
}
