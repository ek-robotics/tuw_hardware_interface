// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_JOINT_DESCRIPTION_H
#define TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_JOINT_DESCRIPTION_H

#include <string>

#include <yaml-cpp/yaml.h>

namespace tuw_ros_control_generic
{
class GenericJointDescription
{
public:
  explicit GenericJointDescription(YAML::Node yaml);
  int getId();
  std::string getName();
private:
  int id_;
  std::string name_;
};
}  // namespace tuw_ros_control_generic

#endif  // TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_JOINT_DESCRIPTION_H
