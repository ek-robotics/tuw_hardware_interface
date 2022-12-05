// Copyright 2022 Eugen Kaltenegger

#ifndef DIP_WS_GENERIC_JOINT_DESCRIPTION_H
#define DIP_WS_GENERIC_JOINT_DESCRIPTION_H

#include <string>

#include <yaml-cpp/yaml.h>

namespace tuw_ros_control_generic
{
class GenericJointDescription {
public:
  explicit GenericJointDescription(YAML::Node yaml);
  int getId();
  std::string getName();
private:
  int id_;
  std::string name_;
};
}

#endif //DIP_WS_GENERIC_JOINT_DESCRIPTION_H
