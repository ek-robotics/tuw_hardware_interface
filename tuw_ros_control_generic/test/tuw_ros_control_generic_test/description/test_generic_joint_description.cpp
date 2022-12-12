// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <tuw_ros_control_generic/description/generic_joint_description.h>

#include "../../tuw_ros_control_generic_test_util/include/file_loader.h"

#define TEST_FILE_PATH "/test/resources/description/test_generic_joint_description.yaml"

using tuw_ros_control_generic_test::FileLoader;
using tuw_ros_control_generic::GenericJointDescription;

class GenericJointDescriptionTest : public ::testing::Test
{
protected:
  GenericJointDescription generic_joint_description_ =
          GenericJointDescription(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));
};

TEST_F(GenericJointDescriptionTest, verifyName)
{
  ASSERT_EQ(generic_joint_description_.getName(), "joint");
}

TEST_F(GenericJointDescriptionTest, verifyId)
{
  ASSERT_EQ(generic_joint_description_.getId(), 0);
}

TEST_F(GenericJointDescriptionTest, verifyConnectionDescriptionPointer)
{
  ASSERT_TRUE(generic_joint_description_.getConnectionDescription());
}

TEST_F(GenericJointDescriptionTest, verifyHardwareDescriptionPointer)
{
  ASSERT_TRUE(generic_joint_description_.getHardwareDescription());
}

TEST_F(GenericJointDescriptionTest, verifyConfigDescriptionPointer)
{
  ASSERT_TRUE(generic_joint_description_.getConfigDescription());
}
