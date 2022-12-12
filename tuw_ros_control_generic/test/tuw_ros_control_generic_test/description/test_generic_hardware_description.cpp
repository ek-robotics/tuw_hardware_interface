// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include "../../tuw_ros_control_generic_test_util/include/file_loader.h"

#include <tuw_ros_control_generic/description/generic_hardware_description.h>

#define TEST_FILE_PATH "/test/resources/description/test_generic_hardware_description.yaml"

using tuw_ros_control_generic_test::FileLoader;
using tuw_ros_control_generic::GenericHardwareDescription;

TEST(TestGenericHardwareDescription, verifyTargetPointer)
{
  GenericHardwareDescription ghd(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_TRUE(ghd.getTargetIdentifierToDescription());
}

TEST(TestGenericHardwareDescription, verifyActualPointer)
{
  GenericHardwareDescription ghd(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_TRUE(ghd.getActualIdentifierToDescription());
}

TEST(TestGenericHardwareDescription, verifyConfigPointer)
{
  GenericHardwareDescription ghd(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_TRUE(ghd.getConfigIdentifierToDescription());
}

TEST(TestGenericHardwareDescription, verifyTargetIdentifier)
{
  GenericHardwareDescription ghd(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_EQ(ghd.getTargetIdentifierToDescription()->size(), 1);
}

TEST(TestGenericHardwareDescription, verifyActualIdentifier)
{
  GenericHardwareDescription ghd(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_EQ(ghd.getActualIdentifierToDescription()->size(), 1);
}

TEST(TestGenericHardwareDescription, verifyConfigIdentifier)
{
  GenericHardwareDescription ghd(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_EQ(ghd.getConfigIdentifierToDescription()->size(), 2);
}