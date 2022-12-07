// Copyright 2022 Eugen Kaltenegger


#include "tuw_ros_control_generic/generic_config.h"

using tuw_ros_control_generic::GenericConfigDescription;
using tuw_ros_control_generic::GenericConfig;

GenericConfig::GenericConfig(std::shared_ptr<GenericConnection> connection,
                             std::shared_ptr<GenericHardware> hardware)
        : connection_(std::move(connection)), hardware_(std::move(hardware))
{
  this->setupReconfigureServer();
}

GenericConfig::GenericConfig(std::shared_ptr<GenericConnection> connection,
                             std::shared_ptr<GenericHardware> hardware,
                             GenericConfigDescription config_description)
        : connection_(std::move(connection)), hardware_(std::move(hardware))
{
  this->setupReconfigureServer();
}

void GenericConfig::setupReconfigureServer()
{

}

void GenericConfig::registerReconfigureVariable(GenericHardwareParameter hardware_parameter)
{

}

void GenericConfig::registerReconfigureEnumVariable(GenericHardwareParameter hardware_parameter)
{

}

void GenericConfig::registerReconfigureRangeVariable(GenericHardwareParameter hardware_parameter)
{

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
