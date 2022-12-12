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
  GenericHardwareDescription generic_hardware_description_ =
          GenericHardwareDescription(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));
};

TEST_F(GenericHardwareDescriptionTest, verifyTargetPointer)
{
  ASSERT_TRUE(generic_hardware_description_.getTargetIdentifierToDescription());
}

TEST_F(GenericHardwareDescriptionTest, verifyActualPointer)
{
  ASSERT_TRUE(generic_hardware_description_.getActualIdentifierToDescription());
}

TEST_F(GenericHardwareDescriptionTest, verifyConfigPointer)
{
  ASSERT_TRUE(generic_hardware_description_.getConfigIdentifierToDescription());
}

TEST_F(GenericHardwareDescriptionTest, verifyTargetIdentifier)
{
  ASSERT_EQ(generic_hardware_description_.getTargetIdentifierToDescription()->size(), 1);
}

TEST_F(GenericHardwareDescriptionTest, verifyActualIdentifier)
{
  ASSERT_EQ(generic_hardware_description_.getActualIdentifierToDescription()->size(), 1);
}

TEST_F(GenericHardwareDescriptionTest, verifyConfigIdentifier)
{
  ASSERT_EQ(generic_hardware_description_.getConfigIdentifierToDescription()->size(), 2);
}
