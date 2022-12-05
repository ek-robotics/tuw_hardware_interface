// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_GENERIC_JOINT_H
#define TUW_ROS_CONTROL_GENERIC_GENERIC_JOINT_H


#include <memory>

#include <ros/package.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_interface.h>

using hardware_interface::JointHandle;
using hardware_interface::JointStateHandle;

using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;

using joint_limits_interface::PositionJointSaturationHandle;
using joint_limits_interface::VelocityJointSaturationHandle;
using joint_limits_interface::EffortJointSaturationHandle;

using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::EffortJointSoftLimitsHandle;

namespace tuw_ros_control_generic
{
class Joint
{
public:
  Joint(GenericJointDescription genericJointDescription,
        GenericConnectionDescription connection_description,
        GenericHardwareDescription hardware_description,
        GenericConfigDescription config_description);

  void write(const ros::Duration &period);
  void read(const ros::Duration &period);

  JointStateHandle* getJointStateHandle();
  JointHandle* getJointPositionHandle();
  JointHandle* getJointVelocityHandle();
  JointHandle* getJointEffortHandle();
private:
  void writeTargetPosition(double* target);
  void writeTargetVelocity(double* target);
  void writeTargetEffort(double* target);

  void readActualPosition(double* target);
  void readActualVelocity(double* target);
  void readActualEffort(double* target);

  GenericJointDescription joint_description_;  // set in constructor
  GenericJointManager joint_manager_;          // set in constructor

  // target values
  double target_position {0.0};
  double target_velocity {0.0};
  double target_effort {0.0};
  // actual values
  double actual_position {0.0};
  double actual_velocity {0.0};
  double actual_effort {0.0};
  // state handle
  std::unique_ptr<JointStateHandle> joint_state_handle {nullptr};
  // command handles
  std::unique_ptr<JointHandle> joint_position_handle {nullptr};
  std::unique_ptr<JointHandle> joint_velocity_handle {nullptr};
  std::unique_ptr<JointHandle> joint_effort_handle {nullptr};
  // limit container
  std::unique_ptr<JointLimits> limits_ {std::make_unique<JointLimits>()};
  std::unique_ptr<SoftJointLimits> soft_limits_ {std::make_unique<SoftJointLimits>()};
  // limit handles
  std::unique_ptr<PositionJointSaturationHandle> joint_position_limit_handle {nullptr};
  std::unique_ptr<VelocityJointSaturationHandle> joint_velocity_limit_handle {nullptr};
  std::unique_ptr<EffortJointSaturationHandle> joint_effort_limit_handle {nullptr};
  std::unique_ptr<PositionJointSoftLimitsHandle> joint_position_soft_limit_handle {nullptr};
  std::unique_ptr<VelocityJointSoftLimitsHandle> joint_velocity_soft_limit_handle {nullptr};
  std::unique_ptr<EffortJointSoftLimitsHandle> joint_effort_soft_limit_handle {nullptr};
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_GENERIC_JOINT_H
