// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_DYNAMIXEL_HARDWARE_INTERFACE_DYNAMIXEL_CONNECTION_H
#define TUW_DYNAMIXEL_HARDWARE_INTERFACE_DYNAMIXEL_CONNECTION_H

#include "tuw_ros_control_generic/generic_connection.h"
#include "description/dynamixel_connection_description.h"

#include <dynamixel_sdk/dynamixel_sdk.h>

#define PROTOCOL "2.0"

using dynamixel::PacketHandler;
using dynamixel::PortHandler;
using tuw_ros_control_generic::GenericConnection;
using tuw_ros_control_generic::GenericConnectionDescription;
using tuw_ros_control_generic::GenericHardwareParameter;

namespace tuw_hardware_interface
{
class DynamixelConnection : public GenericConnection
{
public:
  // singleton function
  static std::shared_ptr<DynamixelConnection> getConnection(const std::shared_ptr<DynamixelConnectionDescription>& connection_description);
  // instance functions
  explicit DynamixelConnection(std::shared_ptr<DynamixelConnectionDescription> connection_description);
  ~DynamixelConnection();

  bool connect() override;
  bool disconnect() override;
  void write(int id, GenericHardwareParameter hardware_parameter, int data) override;
  int read(int id, GenericHardwareParameter hardware_parameter) override;
private:
  static std::unique_ptr<std::map<std::string, std::shared_ptr<DynamixelConnection>>> connection_table_;
  std::shared_ptr<DynamixelConnectionDescription> connection_description_;
  std::unique_ptr<PortHandler> port_handler_;
  std::unique_ptr<PacketHandler> packet_handler_;
};
}

#endif  // TUW_DYNAMIXEL_HARDWARE_INTERFACE_DYNAMIXEL_CONNECTION_H
