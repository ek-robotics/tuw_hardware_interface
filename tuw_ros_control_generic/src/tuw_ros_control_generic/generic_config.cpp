// Copyright 2022 Eugen Kaltenegger

#include <tuw_ros_control_generic/generic_config.h>
#include <tuw_ros_control_generic/generic_connection.h>
#include <tuw_ros_control_generic/generic_hardware.h>
#include <tuw_ros_control_generic/generic_hardware_parameter.h>
#include <tuw_ros_control_generic/generic_joint.h>

using tuw_ros_control_generic::GenericConfig;
using tuw_ros_control_generic::GenericConnection;

GenericConfig::GenericConfig(std::shared_ptr<GenericJoint> joint,
                             std::shared_ptr<GenericHardware> hardware)
        : joint_(std::move(joint)), hardware_(std::move(hardware))
{
  this->setupReconfigureServer();
}

GenericConfig::GenericConfig(std::shared_ptr<GenericJoint> joint,
                             std::shared_ptr<GenericHardware> hardware,
                             GenericConfigDescription config_description)
        : joint_(std::move(joint)), hardware_(std::move(hardware))
{
  this->setupReconfigureServer();
}

void GenericConfig::setupReconfigureServer()
{
  // TODO: get node name
  ros::NodeHandle node_handle(std::string("NODE_NAME") + std::string("/") + this->joint_->getName());
  this->reconfigure_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(node_handle);

  for (const auto& identifier_parameter_pair : *this->hardware_->getConfigIdentifierToParameter())
  {
    auto identifier = identifier_parameter_pair.first;
    auto parameter = identifier_parameter_pair.second;
    this->registerReconfigureVariable(parameter);
    int* present_data = &this->actual_config_values_[identifier];
    this->joint_->read(parameter, present_data);
  }
}

void GenericConfig::registerReconfigureVariable(GenericHardwareParameter hardware_parameter)
{
  if (hardware_parameter.getType() == GenericHardwareParameter::Type::ENUM)
    this->registerReconfigureEnumVariable(hardware_parameter);
  if (hardware_parameter.getType() == GenericHardwareParameter::Type::RANGE)
    this->registerReconfigureRangeVariable(hardware_parameter);
}

void GenericConfig::registerReconfigureEnumVariable(GenericHardwareParameter hardware_parameter)
{
  auto identifier = *hardware_parameter.getIdentifier();
  auto description = *hardware_parameter.getDescription();
  auto enum_map = *hardware_parameter.getEnum();
  int* target = &this->target_config_values_[identifier];
  this->reconfigure_->registerEnumVariable<int>(identifier, target, description, enum_map);
}

void GenericConfig::registerReconfigureRangeVariable(GenericHardwareParameter hardware_parameter)
{
  auto identifier = *hardware_parameter.getIdentifier();
  auto description = *hardware_parameter.getDescription();
  auto minimum = hardware_parameter.getRange()->at("min");
  auto maximum = hardware_parameter.getRange()->at("max");
  int* target = &this->target_config_values_[identifier];
  this->reconfigure_->registerVariable<int>(identifier, target, description, minimum, maximum);
}

void GenericConfig::reconfigureConfig()
{

}

void GenericConfig::reconfigureValue(GenericHardwareParameter hardware_parameter, int target_value)
{

}

void GenericConfig::setInitialConfig(GenericConfigDescription config_description)
{

}
