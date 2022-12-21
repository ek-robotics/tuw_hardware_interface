// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_GENERIC_HARDWARE_PARAMETER_H
#define TUW_ROS_CONTROL_GENERIC_GENERIC_HARDWARE_PARAMETER_H

#include <map>
#include <memory>
#include <string>

#include <tuw_ros_control_generic/description/generic_hardware_parameter_description.h>

namespace tuw_ros_control_generic
{
class GenericHardwareParameter
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
  std::shared_ptr<std::string> getIdentifier();
  std::shared_ptr<std::string> getDescription();
  std::shared_ptr<int> getAddress();
  std::shared_ptr<int> getLength();
  std::shared_ptr<std::map<std::string, int>> getEnum();
  std::shared_ptr<std::map<std::string, int>> getRange();
  bool operator==(const GenericHardwareParameter& other) const;
protected:
  bool isValid();
  bool isTarget();
  bool isActual();
  bool isRange();
  bool isEnum();
  Type type_;
  std::shared_ptr<std::string> identifier_ {nullptr};
  std::shared_ptr<std::string> description_ {nullptr};
  std::shared_ptr<int> address_ {nullptr};
  std::shared_ptr<int> length_ {nullptr};
  std::shared_ptr<std::map<std::string, int>> enum_ {nullptr};
  std::shared_ptr<std::map<std::string, int>> range_ {nullptr};
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_GENERIC_HARDWARE_PARAMETER_H
