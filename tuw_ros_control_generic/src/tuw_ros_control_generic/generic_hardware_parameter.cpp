// Copyright 2022 Eugen Kaltenegger
#include "tuw_ros_control_generic/generic_hardware_parameter.h"

#include <utility>

using tuw_ros_control_generic::GenericHardwareParameter;
using tuw_ros_control_generic::GenericHardwareParameterDescription;

GenericHardwareParameter::GenericHardwareParameter(GenericHardwareParameterDescription hardware_parameter_description)
        : GenericHardwareParameterDescription(std::move(hardware_parameter_description))
{
  if(!this->isValid())
    throw std::runtime_error("parameter " + *this->getIdentifier() + " is invalid" );

  if (this->isTarget()) this->type_ = Type::TARGET;
  if (this->isActual()) this->type_ = Type::ACTUAL;
  if (this->isEnum()  ) this->type_ = Type::ENUM;
  if (this->isRange() ) this->type_ = Type::RANGE;
}

GenericHardwareParameter::Type GenericHardwareParameter::getType()
{
  return this->type_;
}

bool tuw_ros_control_generic::GenericHardwareParameter::isValid()
{
  return this->isTarget() ^ this->isActual() ^ this->isEnum() ^ this->isRange();
}

bool GenericHardwareParameter::isTarget()
{
  return this->getIdentifier() &&
         this->getAddress() &&
         this->getLength() &&
         this->getRange() &&
         !this->getEnum();
}

bool GenericHardwareParameter::isActual()
{
  return this->getIdentifier() &&
         this->getAddress() &&
         this->getLength() &&
         !this->getRange() &&
         !this->getEnum();
}

bool GenericHardwareParameter::isEnum()
{
  return this->getIdentifier() &&
         this->getDescription() &&
         this->getAddress() &&
         this->getLength() &&
         this->getEnum() &&
         !this->getRange();
}

bool GenericHardwareParameter::isRange()
{
  return this->getIdentifier() &&
         this->getDescription() &&
         this->getAddress() &&
         this->getLength() &&
         this->getRange() &&
         !this->getEnum();
}
