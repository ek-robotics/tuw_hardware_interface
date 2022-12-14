// Copyright 2022 Eugen Kaltenegger

// TODO

#include <tuw_ros_control_generic/description/generic_joint_description.h>
#include <tuw_ros_control_generic/generic_connection.h>
#include <tuw_ros_control_generic/generic_hardware.h>
#include <tuw_ros_control_generic/generic_joint.h>
#include <tuw_ros_control_generic/generic_setup_prefix.h>

#include <utility>

using tuw_ros_control_generic::GenericConnection;
using tuw_ros_control_generic::GenericHardware;
using tuw_ros_control_generic::GenericHardwareParameter;
using tuw_ros_control_generic::GenericJoint;
using tuw_ros_control_generic::GenericJointDescription;

GenericJoint::GenericJoint(GenericJointDescription joint_description)
{

}

std::string GenericJoint::getName()
{
  return this->name_;
}

int GenericJoint::getId()
{
  return this->id_;
}

void GenericJoint::write(const ros::Duration &period)
{

}

void GenericJoint::read(const ros::Duration &period)
{

}

void GenericJoint::write(GenericHardwareParameter hardware_parameter, int data)
{
  this->connection_->write(this->id_, std::move(hardware_parameter), data);
}

int GenericJoint::read(GenericHardwareParameter hardware_parameter)
{
  return this->connection_->read(this->id_, std::move(hardware_parameter));
}

JointStateHandle *tuw_ros_control_generic::GenericJoint::getJointStateHandle()
{
  return this->joint_state_handle_.get();
}

JointHandle *tuw_ros_control_generic::GenericJoint::getJointPositionHandle()
{
  return this->joint_position_handle_.get();
}

JointHandle *tuw_ros_control_generic::GenericJoint::getJointVelocityHandle()
{
  return this->joint_velocity_handle_.get();
}

JointHandle *tuw_ros_control_generic::GenericJoint::getJointEffortHandle()
{
  return this->joint_effort_handle_.get();
}

void GenericJoint::writeTarget(int* target, GenericHardware::Mode mode, std::string mode_name)
{
  if (this->hardware_->supportsTargetMode(mode))
    this->connection_->write(this->id_, this->hardware_->getTargetParameterForMode(mode), *target);
  else
    ROS_WARN("[%s] %s is not supporting target mode %s", PREFIX, this->hardware_->getName().LOG, mode_name.LOG);
}

void tuw_ros_control_generic::GenericJoint::writeTargetPosition(double *target)
{
  int hardware_target = static_cast<int>(*target);
  this->writeTarget(&hardware_target, GenericHardware::Mode::POSITION, "POSITION");
}

void tuw_ros_control_generic::GenericJoint::writeTargetVelocity(double *target)
{

}

void tuw_ros_control_generic::GenericJoint::writeTargetEffort(double *target)
{

}

void tuw_ros_control_generic::GenericJoint::readActualPosition(double *target)
{

}

void tuw_ros_control_generic::GenericJoint::readActualVelocity(double *target)
{

}

void tuw_ros_control_generic::GenericJoint::readActualEffort(double *target)
{

}
