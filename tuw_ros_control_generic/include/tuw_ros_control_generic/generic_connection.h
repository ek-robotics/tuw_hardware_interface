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
  explicit GenericConnection(GenericConnectionDescription connection_description);
  virtual ~GenericConnection();
  static std::shared_ptr<GenericConnection> getConnection(GenericConnectionDescription connection_description);
  virtual bool connect() = 0;
  virtual bool disconnect() = 0;
  virtual bool write(int id, GenericHardwareParameter hardware_parameter, int* data) = 0;
  virtual bool read(int id, GenericHardwareParameter hardware_parameter, int* data) = 0;
protected:
  std::mutex connection_mutex_;
  static std::mutex mutex_;
  static std::unique_ptr<std::map<std::string, std::shared_ptr<GenericConnection>>> connection_table_;
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_GENERIC_CONNECTION_H
