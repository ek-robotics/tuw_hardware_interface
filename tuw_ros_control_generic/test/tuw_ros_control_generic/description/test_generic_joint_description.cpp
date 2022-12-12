// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include "../tool/file_loader.cpp"

#include <tuw_ros_control_generic/description/generic_joint_description.h>

using tuw_ros_control_generic::GenericJointDescription;

#define TEST_FILE_PATH "/test/resources/description/test_generic_joint_description.yaml"

TEST(TestGenericJointDescription, verifyName)
{
  GenericJointDescription gjd(loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_EQ(gjd.getName(), "joint");
}

TEST(TestGenericJointDescription, verifyId)
{
  GenericJointDescription gjd(loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_EQ(gjd.getId(), 0);
}

TEST(TestGenericJointDescription, verifyConnectionDescriptionPointer)
{
  GenericJointDescription gjd(loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_TRUE(gjd.getConnectionDescription());
}

TEST(TestGenericJointDescription, verifyHardwareDescriptionPointer)
{
  GenericJointDescription gjd(loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_TRUE(gjd.getHardwareDescription());
}

TEST(TestGenericJointDescription, verifyConfigDescriptionPointer)
{
  GenericJointDescription gjd(loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_TRUE(gjd.getConfigDescription());
}