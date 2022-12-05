// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_GENERIC_CONNECTION_DESCRIPTION_H
#define DIP_WS_GENERIC_CONNECTION_DESCRIPTION_H

#include <yaml-cpp/yaml.h>

namespace tuw_ros_control_generic
{
class GenericConnectionDescription {
public:
  explicit GenericConnectionDescription(YAML::Node yaml);
  std::string getHash();
  std::string getPort();
  int getBaudrate();
private:
  std::string hash_;
  std::string port_;
  int baudrate_;
};
}

#endif //DIP_WS_GENERIC_CONNECTION_DESCRIPTION_H
