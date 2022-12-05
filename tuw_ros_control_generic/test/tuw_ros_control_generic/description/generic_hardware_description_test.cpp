// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>
#include <ros/package.h>


#include "tuw_ros_control_generic/description/generic_hardware_description.h"

using tuw_ros_control_generic::GenericHardwareDescription;

TEST(GenericCHardwareDescriptionTest, verifyConstructorFromYaml)
{
  std::string path = ros::package::getPath("tuw_ros_control_generic") + "/test/resources/generic_hardware_description_test.yaml";
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
