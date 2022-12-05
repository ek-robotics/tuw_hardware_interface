// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <tuw_ros_control_generic/description/generic_connection_description.h>

using tuw_ros_control_generic::GenericConnectionDescription;

TEST(GenericConnectionDescriptionTest, verifyConstructorFromYaml)
{
  YAML::Node yaml = YAML::Load("{port: \"/dev/ttyUSB0\", baudrate: 57600}");
  GenericConnectionDescription gcd(yaml);
  ASSERT_EQ(gcd.getHash(), "/dev/ttyUSB0_57600");
  ASSERT_EQ(gcd.getPort(), "/dev/ttyUSB0");
  ASSERT_EQ(gcd.getBaudrate(), 57600);
}
