// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_GENERIC_CONNECTION_H
#define TUW_ROS_CONTROL_GENERIC_GENERIC_CONNECTION_H

#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace tuw_ros_control_generic
{
class GenericHardwareParameter;
class GenericConnectionDescription;
class GenericConnection
{
public:
  // singleton function
  static std::shared_ptr<GenericConnection> getConnection(GenericConnectionDescription connection_description);
  // instance functions
  explicit GenericConnection(GenericConnectionDescription connection_description);
  ~GenericConnection();
  virtual bool connect() = 0;
  virtual bool disconnect() = 0;
  virtual void write(int id, GenericHardwareParameter hardware_parameter, int data) = 0;
  virtual int read(int id, GenericHardwareParameter hardware_parameter) = 0;
protected:
  // singleton variables
  static std::mutex mutex_;
  static std::unique_ptr<std::map<std::string, std::shared_ptr<GenericConnection>>> connection_table_;
  // instance variables
  std::mutex connection_mutex_;
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_GENERIC_CONNECTION_H
