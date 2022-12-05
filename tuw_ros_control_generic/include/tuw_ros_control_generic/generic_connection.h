// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_GENERIC_CONNECTION_H
#define TUW_ROS_CONTROL_GENERIC_GENERIC_CONNECTION_H

#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace tuw_ros_control_generic
{
class GenericConnection
{
public:
  static std::shared_ptr<GenericConnection> getConnection(GenericConnectionDescpription connection_description);
  bool connect();
  bool disconnect();
  virtual bool write(int id, GenericHardwareParameter hardware_parameter, int* data);
  virtual bool read(int id, GenericHardwareParameter hardware_parameter, int* data);
private:
  std::mutex mutex_;
  static std::map<std::string, GenericConnection> connection_table_;
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_GENERIC_CONNECTION_H
