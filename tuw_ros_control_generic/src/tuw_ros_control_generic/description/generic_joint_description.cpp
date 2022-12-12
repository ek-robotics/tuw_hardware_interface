// Copyright 2022 Eugen Kaltenegger

#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <tuw_ros_control_generic/description/generic_config_description.h>
#include <tuw_ros_control_generic/description/generic_connection_description.h>
#include <tuw_ros_control_generic/description/generic_hardware_description.h>
#include <tuw_ros_control_generic/description/generic_joint_description.h>

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
    // TODO: add node name to print
    ROS_ERROR("missing connection description");

  if (yaml["hardware"].IsDefined())
    this->hardware_description_ = std::make_shared<GenericHardwareDescription>(loadFile(yaml["hardware"]));
  else
    // TODO: add node name to print
    ROS_ERROR("missing hardware description");

  if (yaml["config"].IsDefined())
    this->config_description_ = std::make_shared<GenericConfigDescription>(loadFile(yaml["config"]));
  else
    // TODO: add node name to print
    ROS_INFO("no config provided - fallback to present hardware config");
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

YAML::Node GenericJointDescription::loadFile(YAML::Node yaml)
{
  std::string path = ros::package::getPath(yaml["package"].as<std::string>() + "/" + yaml["path"].as<std::string>());
  return YAML::LoadFile(path);
}
