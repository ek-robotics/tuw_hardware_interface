// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_TRINAMIC_ROS_CONTROL_TRINAMIC_CONNECTION_H
#define TUW_TRINAMIC_ROS_CONTROL_TRINAMIC_CONNECTION_H

#include <fcntl.h>
#include <cerrno>
#include <termios.h>
#include <memory>
#include <cstring>
#include <string>
#include <unistd.h>

#include "tuw_hardware_interface_template/generic_connection.h"

#define SET_AXIS_PARAMETER 5
#define GET_AXIS_PARAMETER 6

#define SUCCESS 100
#define WRONG_CHECKSUM 1
#define INVALID_COMMAND 2
#define WRONG_TYPE 3
#define INVALID_VALUE 4
#define CONFIGURATION_LOCKED 5
#define COMMAND_NOT_AVAILABLE 6

namespace tuw_hardware_interface
{
class TrinamicCommand;
class TrinamicReply;
class TMCM1640Connection : public GenericConnection
{
public:
  static std::shared_ptr<TMCM1640Connection> getConnection
          (const std::shared_ptr<GenericConnectionDescription>& connection_description);

  explicit TMCM1640Connection(std::shared_ptr<GenericConnectionDescription> connection_description);
  ~TMCM1640Connection();

  bool connect() override;
  bool disconnect() override;

  void write(int id, GenericHardwareParameter hardware_parameter, int data) override;
  int read(int id, GenericHardwareParameter hardware_parameter) override;

private:
  TrinamicReply communicate(TrinamicCommand command);

  std::shared_ptr<GenericConnectionDescription> connection_description_;
  int serial_port_{};
  struct termios tty_{};
};
}  // namespace tuw_trinamic_ros_control

#endif  // TUW_TRINAMIC_ROS_CONTROL_TRINAMIC_CONNECTION_H
