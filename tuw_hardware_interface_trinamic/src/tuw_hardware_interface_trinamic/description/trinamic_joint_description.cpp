// Copyright 2022 Eugen Kaltenegger

#include <tuw_hardware_interface_trinamic/description/trinamic_joint_description.h>
#include <tuw_hardware_interface_template/generic_setup_prefix.h>

using tuw_hardware_interface::TrinamicHardwareDescription;
using tuw_hardware_interface::TrinamicJointDescription;

TrinamicJointDescription::TrinamicJointDescription(const YAML::Node& yaml)
{
  this->id_ = yaml["id"].as<int>();
  this->name_ = yaml["name"].as<std::string>();

  if (yaml["connection"].IsDefined())
    this->connection_description_ =
            std::make_shared<GenericConnectionDescription>(yaml["connection"]);
  else
    ROS_ERROR("[%s] missing connection description", PREFIX);

  if (yaml["hardware"].IsDefined())
  {
    YAML::Node hardware_yaml = loadFile(yaml["hardware"]);
    this->hardware_description_ = std::make_shared<GenericHardwareDescription>(hardware_yaml);
  }
  else
    ROS_ERROR("[%s] missing hardware description", PREFIX);

  if (yaml["config"].IsDefined())
    this->config_description_ =
            std::make_shared<GenericConfigDescription>(loadFile(yaml["config"]));
  else
    ROS_INFO("[%s] no config provided - fallback to present hardware config", PREFIX);
}

std::shared_ptr<TrinamicHardwareDescription>
TrinamicJointDescription::getTrinamicHardwareDescription()
{
  return this->trinamic_hardware_description_;
}


