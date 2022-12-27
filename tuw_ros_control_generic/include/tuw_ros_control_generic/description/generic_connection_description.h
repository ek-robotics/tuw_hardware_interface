// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_CONNECTION_DESCRIPTION_H
#define TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_CONNECTION_DESCRIPTION_H

#include <string>

#include <yaml-cpp/yaml.h>

namespace tuw_ros_control_generic
{
class GenericConnectionDescription
{
public:
  explicit GenericConnectionDescription(YAML::Node yaml);
  std::string getHash();
  std::string getPort();
  int getBaudrate() const;
private:
  std::string hash_;
  std::string port_;
  int baudrate_;
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_CONNECTION_DESCRIPTION_H
