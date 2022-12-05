// Copyright 2022 Eugen Kaltenegger

#include <tuw_ros_control_generic/description/generic_hardware_description.h>

#include <string>

#include <ros/package.h>

#include <gtest/gtest.h>

using tuw_ros_control_generic::GenericHardwareDescription;

#define RELATIVE_PATH "/test/resources/generic_hardware_description_test.yaml"

TEST(GenericCHardwareDescriptionTest, verifyConstructorFromYaml)
{
  std::string path = ros::package::getPath("tuw_ros_control_generic") + std::string(RELATIVE_PATH);
  YAML::Node yaml = YAML::LoadFile(path);
  GenericHardwareDescription ghd(yaml);

  ASSERT_TRUE(ghd.getTargetValues());
  ASSERT_TRUE(ghd.getActualValues());
  ASSERT_TRUE(ghd.getConfigValues());

  ASSERT_EQ(*ghd.getTargetValues()->at("tsp").getIdentifier(), "tsp");
  ASSERT_EQ(*ghd.getActualValues()->at("asp").getIdentifier(), "asp");
  ASSERT_EQ(*ghd.getConfigValues()->at("ecp").getIdentifier(), "ecp");
  ASSERT_EQ(*ghd.getConfigValues()->at("mcp").getIdentifier(), "mcp");
}
