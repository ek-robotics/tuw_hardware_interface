// Copyright 2023 Eugen Kaltenegger

#include "tuw_hardware_interface_trinamic/trinamic_joint.h"
#include "tuw_hardware_interface_template/description/generic_joint_description.h"
#include "tuw_hardware_interface_template/generic_setup_prefix.h"
#include "tuw_hardware_interface_trinamic/trinamic_hardware.h"

using tuw_hardware_interface::TrinamicJoint;

void TrinamicJoint::setTrinamicConnection(std::shared_ptr<TMCM1640Connection> connection)
{
  this->trinamic_connection_ = connection;
}

void TrinamicJoint::writeTrinamic(TrinamicHardwareParameter hardware_parameter, int data)
{
  this->trinamic_connection_->writeTrinamic(this->id_, hardware_parameter, data);
}

int TrinamicJoint::readTrinamic(TrinamicHardwareParameter hardware_parameter)
{
  return this->trinamic_connection_->readTrinamic(this->id_, hardware_parameter);
}

tuw_hardware_interface::TrinamicJoint::TrinamicJoint(GenericJointDescription joint_description)
        : GenericJoint(joint_description)
{

}

void tuw_hardware_interface::TrinamicJoint::writeTarget(double target, GenericHardware::Mode mode,
                                                        const std::string& mode_name)
{
  if (this->hardware_->supportsTargetMode(mode))
  {
    int hardware_target = this->hardware_->convertToHardwareResolution(target, mode);
    this->trinamic_connection_->writeTrinamic(this->id_, this->hardware_->getTargetTrinamicParameterForMode(mode), hardware_target);
  }
  else
  {
    ROS_WARN("[%s] %s is not supporting target mode %s", PREFIX, this->hardware_->getName().LOG, mode_name.LOG);
    throw std::runtime_error("unsupported target mode requested");
  }

}

double tuw_hardware_interface::TrinamicJoint::readActual(GenericHardware::Mode mode, const std::string& mode_name)
{
  if (this->hardware_->supportsActualMode(mode))
  {
    int hardware_actual = this->trinamic_connection_->readTrinamic(this->id_, this->hardware_->getActualTrinamicParameterForMode(mode));
    return this->hardware_->convertFromHardwareResolution(hardware_actual, mode);
  }
  else
  {
    ROS_WARN("[%s] %s is not supporting target mode %s", PREFIX, this->hardware_->getName().LOG, mode_name.LOG);
    throw std::runtime_error("unsupported actual mode requested");
  }
}

void TrinamicJoint::setTrinamicHardware(std::shared_ptr<TrinamicHardware> hardware)
{
  GenericJoint::hardware_ = hardware;
  this->hardware_ = hardware;
}
