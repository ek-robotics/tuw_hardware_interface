// Copyright 2022 Eugen Kaltenegger

#include <memory>
#include <string>
#include <utility>

#include <tuw_ros_control_generic/description/generic_joint_description.h>
#include <tuw_ros_control_generic/generic_connection.h>
#include <tuw_ros_control_generic/generic_hardware.h>
#include <tuw_ros_control_generic/generic_joint.h>
#include <tuw_ros_control_generic/generic_setup_prefix.h>

#include <urdf/model.h>

using tuw_ros_control_generic::GenericConnection;
using tuw_ros_control_generic::GenericHardware;
using tuw_ros_control_generic::GenericHardwareParameter;
using tuw_ros_control_generic::GenericJoint;
using tuw_ros_control_generic::GenericJointDescription;

GenericJoint::GenericJoint(GenericJointDescription joint_description)
{
  this->name_ = joint_description.getName();
  this->id_ = joint_description.getId();

  this->joint_state_handle_ = std::make_unique<JointStateHandle>
          (this->name_, &this->actual_position_, &this->actual_velocity_, &this->actual_effort_);

  this->joint_position_handle_ = std::make_unique<JointHandle>
          (*this->joint_state_handle_, &this->target_position_);
  this->joint_velocity_handle_ = std::make_unique<JointHandle>
          (*this->joint_state_handle_, &this->target_velocity_);
  this->joint_effort_handle_ = std::make_unique<JointHandle>
          (*this->joint_state_handle_, &this->target_effort_);

  urdf::Model urdf_model;
  urdf_model.initParam("/robot_description");

  const bool has_limits      = getJointLimits(urdf_model.getJoint(this->name_), *this->limits_);
  const bool has_soft_limits = getSoftJointLimits(urdf_model.getJoint(this->name_), *this->soft_limits_);

  if (has_limits && has_soft_limits)
  {
    this->joint_position_soft_limit_handle_ = std::make_unique<PositionJointSoftLimitsHandle>
            (*this->joint_position_handle_, *this->limits_, *this->soft_limits_);
    this->joint_velocity_soft_limit_handle_ = std::make_unique<VelocityJointSoftLimitsHandle>
            (*this->joint_velocity_handle_, *this->limits_, *this->soft_limits_);
    this->joint_effort_soft_limit_handle_ = std::make_unique<EffortJointSoftLimitsHandle>
            (*this->joint_effort_handle_, *this->limits_, *this->soft_limits_);
  }

  if (has_limits && !has_soft_limits)
  {
    this->joint_position_limit_handle_ = std::make_unique<PositionJointSaturationHandle>
            (*this->joint_position_handle_, *this->limits_);
    this->joint_velocity_limit_handle_ = std::make_unique<VelocityJointSaturationHandle>
            (*this->joint_velocity_handle_, *this->limits_);
    this->joint_effort_limit_handle_ = std::make_unique<EffortJointSaturationHandle>
            (*this->joint_effort_handle_, *this->limits_);
  }
}

void GenericJoint::setConnection(std::shared_ptr<GenericConnection> connection)
{
  this->connection_ = std::move(connection);
}

void GenericJoint::setHardware(std::shared_ptr<GenericHardware> hardware)
{
  this->hardware_ = std::move(hardware);
}

void GenericJoint::setConfig(std::shared_ptr<GenericConfig> config)
{
  this->config_ = std::move(config);
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
  if (!this->mode_)
  {
    ROS_DEBUG("[%s] operation mode for joint %s is (yet?) not defined", PREFIX, this->name_.LOG);
    return;
  }

  switch (*this->mode_)
  {
    case GenericHardware::Mode::POSITION:
      if (this->joint_position_soft_limit_handle_ != nullptr)
        this->joint_position_soft_limit_handle_->enforceLimits(period);
      else if (this->joint_position_limit_handle_ != nullptr)
        this->joint_position_limit_handle_->enforceLimits(period);
      this->writeTargetPosition(this->target_position_);
      break;
    case GenericHardware::Mode::VELOCITY:
      if (this->joint_velocity_soft_limit_handle_ != nullptr)
        this->joint_velocity_soft_limit_handle_->enforceLimits(period);
      else if (this->joint_velocity_limit_handle_ != nullptr)
        this->joint_velocity_limit_handle_->enforceLimits(period);
      this->writeTargetVelocity(this->target_velocity_);
      break;
    case GenericHardware::Mode::EFFORT:
      if (this->joint_effort_soft_limit_handle_ != nullptr)
        this->joint_effort_soft_limit_handle_->enforceLimits(period);
      else if (this->joint_effort_limit_handle_ != nullptr)
        this->joint_effort_limit_handle_->enforceLimits(period);
      this->writeTargetEffort(this->target_effort_);
      break;
  }
}

void GenericJoint::read(const ros::Duration &period)
{
  if (this->hardware_->supportsActualMode(GenericHardware::Mode::POSITION))
  {
    this->actual_position_ = this->readActualPosition();
  }

  if (this->hardware_->supportsActualMode(GenericHardware::Mode::VELOCITY))
  {
    this->actual_velocity_ = this->readActualVelocity();
  }

  if (this->hardware_->supportsActualMode(GenericHardware::Mode::EFFORT))
  {
    this->actual_effort_ = this->readActualEffort();
  }
}

void GenericJoint::write(GenericHardwareParameter hardware_parameter, int data)
{
  this->connection_->write(this->id_, std::move(hardware_parameter), data);
}

int GenericJoint::read(GenericHardwareParameter hardware_parameter)
{
  return this->connection_->read(this->id_, std::move(hardware_parameter));
}

bool GenericJoint::setMode(GenericHardware::Mode mode)
{
  if (this->hardware_->supportsTargetMode(mode))
  {
    this->mode_ = std::make_unique<GenericHardware::Mode>(mode);
    ROS_INFO("[%s] SUCCESS setting mode %s on joint %s",
             PREFIX, GenericHardware::modeToString(mode).LOG, this->name_.LOG);
    return true;
  }
  else
  {
    ROS_WARN("[%s] ERROR setting mode %s on joint %s - this mode is not supported by %s",
             PREFIX, GenericHardware::modeToString(mode).LOG, this->name_.LOG, this->hardware_->getName().LOG);
    return false;
  }
}

JointStateHandle *GenericJoint::getJointStateHandle()
{
  return this->joint_state_handle_.get();
}

JointHandle *GenericJoint::getJointPositionHandle()
{
  return this->joint_position_handle_.get();
}

JointHandle *GenericJoint::getJointVelocityHandle()
{
  return this->joint_velocity_handle_.get();
}

JointHandle *GenericJoint::getJointEffortHandle()
{
  return this->joint_effort_handle_.get();
}

void GenericJoint::writeTarget(double target, GenericHardware::Mode mode, const std::string& mode_name)
{
  if (this->hardware_->supportsTargetMode(mode))
  {
    int hardware_target = this->hardware_->convertToHardwareResolution(target, mode);
    this->connection_->write(this->id_, this->hardware_->getTargetParameterForMode(mode), hardware_target);
  }
  else
  {
    ROS_WARN("[%s] %s is not supporting target mode %s", PREFIX, this->hardware_->getName().LOG, mode_name.LOG);
    throw std::runtime_error("unsupported target mode requested");
  }
}

void GenericJoint::writeTargetPosition(double target)
{
  this->writeTarget(target, GenericHardware::Mode::POSITION, "POSITION");
}

void GenericJoint::writeTargetVelocity(double target)
{
  this->writeTarget(target, GenericHardware::Mode::VELOCITY, "VELOCITY");
}

void GenericJoint::writeTargetEffort(double target)
{
  this->writeTarget(target, GenericHardware::Mode::EFFORT, "EFFORT");
}

double GenericJoint::readActual(GenericHardware::Mode mode, const std::string& mode_name)
{
  if (this->hardware_->supportsActualMode(mode))
  {
    int hardware_actual = this->connection_->read(this->id_, this->hardware_->getActualParameterForMode(mode));
    return this->hardware_->convertFromHardwareResolution(hardware_actual, mode);
  }
  else
  {
    ROS_WARN("[%s] %s is not supporting target mode %s", PREFIX, this->hardware_->getName().LOG, mode_name.LOG);
    throw std::runtime_error("unsupported actual mode requested");
  }
}

double GenericJoint::readActualPosition()
{
  return this->readActual(GenericHardware::Mode::POSITION, "POSITION");
}

double GenericJoint::readActualVelocity()
{
  return this->readActual(GenericHardware::Mode::VELOCITY, "VELOCITY");
}

double GenericJoint::readActualEffort()
{
  return this->readActual(GenericHardware::Mode::EFFORT, "EFFORT");
}
