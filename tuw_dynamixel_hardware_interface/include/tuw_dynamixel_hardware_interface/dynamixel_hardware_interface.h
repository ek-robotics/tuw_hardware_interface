// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_DYNAMIXEL_HARDWARE_INTERFACE_H
#define DIP_WS_DYNAMIXEL_HARDWARE_INTERFACE_H

#include "tuw_ros_control_generic/generic_hardware_interface.h"
#include "description/dynamixel_joint_description.h"

using tuw_ros_control_generic::GenericHardwareInterface;

namespace tuw_hardware_interface
{
class DynamixelHardwareInterface : public GenericHardwareInterface
{
public:
  bool init(ros::NodeHandle &basic_node_handle, ros::NodeHandle &hardware_node_handle) override;
private:
  bool initJoints(std::list<DynamixelJointDescription> joint_descriptions);
  bool initJoint(DynamixelJointDescription joint_description);
};
}

#endif //DIP_WS_DYNAMIXEL_HARDWARE_INTERFACE_H
