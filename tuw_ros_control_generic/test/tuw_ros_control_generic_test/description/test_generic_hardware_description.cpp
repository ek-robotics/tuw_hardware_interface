// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <tuw_ros_control_generic/description/generic_hardware_description.h>

#include "../../tuw_ros_control_generic_test_util/include/file_loader.h"

#define TEST_FILE_PATH "/test/resources/description/test_generic_hardware_description.yaml"

using tuw_ros_control_generic_test::FileLoader;
using tuw_ros_control_generic::GenericHardwareDescription;

class GenericHardwareDescriptionTest : public ::testing::Test
{
protected:
  GenericHardwareDescription defined_resolution_generic_hardware_description_ =
          GenericHardwareDescription(FileLoader::loadYAMLFromFile(TEST_FILE_PATH)["defined_resolution"]);
  GenericHardwareDescription undefined_resolution_generic_hardware_description_ =
          GenericHardwareDescription(FileLoader::loadYAMLFromFile(TEST_FILE_PATH)["undefined_resolution"]);
};

TEST_F(GenericHardwareDescriptionTest, verifyName)
{
  ASSERT_EQ(defined_resolution_generic_hardware_description_.getName(), "generic_hardware");
}

TEST_F(GenericHardwareDescriptionTest, verifyDefniedPositionResolution)
{
  ASSERT_TRUE(defined_resolution_generic_hardware_description_.getPositionResolution());
  ASSERT_EQ(*defined_resolution_generic_hardware_description_.getPositionResolution(), 0.5);
}

TEST_F(GenericHardwareDescriptionTest, verifyyDefniedVelocityResolution)
{
  ASSERT_TRUE(defined_resolution_generic_hardware_description_.getPositionResolution());
  ASSERT_EQ(*defined_resolution_generic_hardware_description_.getPositionResolution(), 0.5);
}

TEST_F(GenericHardwareDescriptionTest, verifyyDefniedEffortResolution)
{
  ASSERT_TRUE(defined_resolution_generic_hardware_description_.getPositionResolution());
  ASSERT_EQ(*defined_resolution_generic_hardware_description_.getPositionResolution(), 0.5);
}

TEST_F(GenericHardwareDescriptionTest, verifyUndefinedPositionResolution)
{
  ASSERT_FALSE(undefined_resolution_generic_hardware_description_.getPositionResolution());
}

TEST_F(GenericHardwareDescriptionTest, verifyUndefinedVelocityResolution)
{
  ASSERT_FALSE(undefined_resolution_generic_hardware_description_.getPositionResolution());
}

TEST_F(GenericHardwareDescriptionTest, verifyUndefinedEffortResolution)
{
  ASSERT_FALSE(undefined_resolution_generic_hardware_description_.getPositionResolution());
}

TEST_F(GenericHardwareDescriptionTest, verifyTargetPointer)
{
  ASSERT_TRUE(defined_resolution_generic_hardware_description_.getTargetIdentifierToDescription());
}

TEST_F(GenericHardwareDescriptionTest, verifyTargetIdentifier)
{
  ASSERT_EQ(defined_resolution_generic_hardware_description_.getTargetIdentifierToDescription()->size(), 3);
}

TEST_F(GenericHardwareDescriptionTest, verifyActualPointer)
{
  ASSERT_TRUE(defined_resolution_generic_hardware_description_.getActualIdentifierToDescription());
}

TEST_F(GenericHardwareDescriptionTest, verifyActualIdentifier)
{
  ASSERT_EQ(defined_resolution_generic_hardware_description_.getActualIdentifierToDescription()->size(), 3);
}

TEST_F(GenericHardwareDescriptionTest, verifyConfigPointer)
{
  ASSERT_TRUE(defined_resolution_generic_hardware_description_.getConfigIdentifierToDescription());
}

TEST_F(GenericHardwareDescriptionTest, verifyConfigIdentifier)
{
  ASSERT_EQ(defined_resolution_generic_hardware_description_.getConfigIdentifierToDescription()->size(), 2);
}
