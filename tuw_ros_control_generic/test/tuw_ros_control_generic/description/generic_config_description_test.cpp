// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <tuw_ros_control_generic/description/generic_config_description.h>

using tuw_ros_control_generic::GenericConfigDescription;

TEST(GenericConfigDescriptionTest, verifyConstructorFromYaml)
{
  YAML::Node yaml = YAML::Load("{config_value_1: 1, config_value_2: 2}");
  GenericConfigDescription gcd(yaml);
  ASSERT_EQ(gcd.getConfigMap().at("config_value_1"), 1);
  ASSERT_EQ(gcd.getConfigMap().at("config_value_2"), 2);
}
