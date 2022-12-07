// Copyright 2022 Eugen Kaltenegger

#include <tuw_ros_control_generic/description/generic_hardware_description.h>

#include <string>

#include <ros/package.h>

#include <gtest/gtest.h>

using tuw_ros_control_generic::GenericHardwareDescription;

#define RELATIVE_PATH "/test/resources/description/test_generic_hardware_description.yaml"

TEST(TestGenericHardwareDescription, verifyConstructorFromYaml)
{
  std::string path = ros::package::getPath("tuw_ros_control_generic") + std::string(RELATIVE_PATH);
  YAML::Node yaml = YAML::LoadFile(path);
  GenericHardwareDescription ghd(yaml);

  ASSERT_TRUE(ghd.getTargetIdentifierToDescription());
  ASSERT_TRUE(ghd.getActualIdentifierToDescription());
  ASSERT_TRUE(ghd.getConfigIdentifierToDescription());

  ASSERT_EQ(*ghd.getTargetIdentifierToDescription()->at("tsp").getIdentifier(), "tsp");
  ASSERT_EQ(*ghd.getActualIdentifierToDescription()->at("asp").getIdentifier(), "asp");
  ASSERT_EQ(*ghd.getConfigIdentifierToDescription()->at("ecp").getIdentifier(), "ecp");
  ASSERT_EQ(*ghd.getConfigIdentifierToDescription()->at("rcp").getIdentifier(), "rcp");
}
