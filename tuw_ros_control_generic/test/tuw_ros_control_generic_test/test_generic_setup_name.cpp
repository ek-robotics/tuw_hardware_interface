// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <tuw_ros_control_generic/generic_setup_name.h>

using tuw_ros_control_generic::GenericSetupName;

TEST(GenericSetupNameTest, verifySetupName)
{
  std::string name = "test_name";
  GenericSetupName::setSetupName(name);
  ASSERT_EQ(GenericSetupName::getSetupName(), name);
}
