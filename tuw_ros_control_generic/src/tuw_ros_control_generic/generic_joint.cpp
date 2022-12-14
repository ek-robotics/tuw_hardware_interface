// Copyright 2022 Eugen Kaltenegger

// TODO

#include "tuw_ros_control_generic/description/generic_joint_description.h"
#include "tuw_ros_control_generic/generic_hardware_parameter.h"
#include "tuw_ros_control_generic/generic_joint.h"

using tuw_ros_control_generic::GenericJoint;
using tuw_ros_control_generic::GenericJointDescription;
using tuw_ros_control_generic::GenericHardwareParameter;

tuw_ros_control_generic::GenericJoint::GenericJoint(GenericJointDescription joint_description)
{

}

std::string tuw_ros_control_generic::GenericJoint::getName()
{
  return std::string();
}

int tuw_ros_control_generic::GenericJoint::getId()
{
  return 0;
}

void tuw_ros_control_generic::GenericJoint::write(const ros::Duration &period)
{

}

void tuw_ros_control_generic::GenericJoint::read(const ros::Duration &period)
{

}

void tuw_ros_control_generic::GenericJoint::write(GenericHardwareParameter hardware_parameter,
                                                  int *data)
{

}

void tuw_ros_control_generic::GenericJoint::read(GenericHardwareParameter hardware_parameter,
                                                 int *data)
{

}
