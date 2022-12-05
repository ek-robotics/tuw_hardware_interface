// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <tuw_ros_control_generic/description/generic_joint_description.h>

using tuw_ros_control_generic::GenericJointDescription;

TEST(GenericJointDescriptionTest, verifyConstructorFromYaml)
{
  YAML::Node yaml = YAML::Load("{name: \"joint\", id: 1}");
  GenericJointDescription gjd(yaml);
  ASSERT_EQ(gjd.getName(), "joint");
  ASSERT_EQ(gjd.getId(), 1);
}
