// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_GENERIC_HARDWARE_PARAMETER_H
#define TUW_ROS_CONTROL_GENERIC_GENERIC_HARDWARE_PARAMETER_H

#include <tuw_ros_control_generic/description/generic_hardware_parameter_description.h>

namespace tuw_ros_control_generic
{
class GenericHardwareParameter : public GenericHardwareParameterDescription
{
public:
  explicit GenericHardwareParameter(GenericHardwareParameterDescription hardware_parameter_description);
  enum Type
  {
    TARGET,
    ACTUAL,
    ENUM,
    RANGE
  };
  Type getType();
protected:
//  GenericHardwareParameterDescription hardware_parameter_description_;
  Type type_;
  bool isValid();
  bool isTarget();
  bool isActual();
  bool isRange();
  bool isEnum();
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_GENERIC_HARDWARE_PARAMETER_H
