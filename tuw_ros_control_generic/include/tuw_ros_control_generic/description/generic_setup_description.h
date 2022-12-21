// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_SETUP_DESCRIPTION_H
#define TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_SETUP_DESCRIPTION_H

#include <list>
#include <string>

#include <yaml-cpp/yaml.h>

namespace tuw_ros_control_generic
{
class GenericJointDescription;
class GenericSetupDescription
{
public:
  explicit GenericSetupDescription(YAML::Node yaml);
  std::string getName();
  std::list<GenericJointDescription> getJoints();
protected:
  std::string name_;
  std::list<GenericJointDescription> joints_ {std::list<GenericJointDescription>()};
};
}  // namespace tuw_ros_control_generic


#endif  // TUW_ROS_CONTROL_GENERIC_DESCRIPTION_GENERIC_SETUP_DESCRIPTION_H
