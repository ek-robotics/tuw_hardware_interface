// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_GENERIC_CONFIG_H
#define TUW_ROS_CONTROL_GENERIC_GENERIC_CONFIG_H

#include <memory>

#include <tuw_ros_control_generic/description/generic_config_description.h>
#include <tuw_ros_control_generic/generic_connection.h>
#include <tuw_ros_control_generic/generic_hardware.h>

namespace tuw_ros_control_generic
{
class GenericConfig
{
public:
  GenericConfig(GenericConfigDescription config_description,
                std::shared_ptr<GenericConnection> connection,
                std::shared_ptr<GenericHardware> hardware);
private:
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_GENERIC_CONFIG_H
