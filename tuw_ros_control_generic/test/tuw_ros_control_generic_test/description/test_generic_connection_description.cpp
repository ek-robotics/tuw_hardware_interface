// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include "../../tuw_ros_control_generic_test_util/include/file_loader.h"

#include <tuw_ros_control_generic/description/generic_connection_description.h>

#define TEST_FILE_PATH "/test/resources/description/test_generic_connection_description.yaml"

using tuw_ros_control_generic_test::FileLoader;
using tuw_ros_control_generic::GenericConnectionDescription;

TEST(TestGenericConnectionDescription, verifyHash)
{
  GenericConnectionDescription gcd(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_EQ(gcd.getHash(), "/dev/ttyUSB0_57600");
}

TEST(TestGenericConnectionDescription, verifyPort)
{
  GenericConnectionDescription gcd(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_EQ(gcd.getPort(), "/dev/ttyUSB0");
}

TEST(TestGenericConnectionDescription, verifyBaudrate)
{
  GenericConnectionDescription gcd(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_EQ(gcd.getBaudrate(), 57600);
}