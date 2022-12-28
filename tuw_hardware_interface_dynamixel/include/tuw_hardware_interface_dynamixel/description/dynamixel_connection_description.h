// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_DYNAMIXEL_CONNECTION_DESCRIPTION_H
#define DIP_WS_DYNAMIXEL_CONNECTION_DESCRIPTION_H

#include <tuw_ros_control_generic/description/generic_connection_description.h>

using tuw_ros_control_generic::GenericConnectionDescription;

namespace tuw_hardware_interface
{
class DynamixelConnectionDescription : public GenericConnectionDescription
{
public:
  explicit DynamixelConnectionDescription(YAML::Node yaml);
  std::string getProtocol() const;
private:
  std::string protocol_;
};
}

#endif //DIP_WS_DYNAMIXEL_CONNECTION_DESCRIPTION_H
