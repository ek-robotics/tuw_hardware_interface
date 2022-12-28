// Copyright 2022 Eugen Kaltenegger

#include <tuw_hardware_interface_template/description/generic_joint_description.h>

#include <memory>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <tuw_hardware_interface_template/description/generic_config_description.h>
#include <tuw_hardware_interface_template/description/generic_connection_description.h>
#include <tuw_hardware_interface_template/description/generic_hardware_description.h>
#include <tuw_hardware_interface_template/generic_setup_prefix.h>

using tuw_ros_control_generic::GenericConfigDescription;
using tuw_ros_control_generic::GenericConnectionDescription;
using tuw_ros_control_generic::GenericHardwareDescription;
using tuw_ros_control_generic::GenericJointDescription;

GenericJointDescription::GenericJointDescription(YAML::Node yaml)
{
  this->id_ = yaml["id"].as<int>();
  this->name_ = yaml["name"].as<std::string>();

  if (yaml["connection"].IsDefined())
    this->connection_description_ = std::make_shared<GenericConnectionDescription>(yaml["connection"]);
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
    this->config_description_ = std::make_shared<GenericConfigDescription>(loadFile(yaml["config"]));
  else
    ROS_INFO("[%s] no config provided - fallback to present hardware config", PREFIX);
}

int GenericJointDescription::getId()
{
  return this->id_;
}

std::string GenericJointDescription::getName()
{
  return this->name_;
}

std::shared_ptr<GenericConnectionDescription> GenericJointDescription::getConnectionDescription()
{
  return this->connection_description_;
}

std::shared_ptr<GenericHardwareDescription> GenericJointDescription::getHardwareDescription()
{
  return this->hardware_description_;
}

std::shared_ptr<GenericConfigDescription> GenericJointDescription::getConfigDescription()
{
  return this->config_description_;
}

YAML::Node GenericJointDescription::loadFile(YAML::Node location_yaml)
{
  std::string path = ros::package::getPath(location_yaml["package"].as<std::string>()) + "/" + location_yaml["path"].as<std::string>();
  YAML::Node yaml =  YAML::LoadFile(path);
  return yaml;
}
