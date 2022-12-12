// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <tuw_ros_control_generic/description/generic_connection_description.h>

#include "../../tuw_ros_control_generic_test_util/include/file_loader.h"

#define TEST_FILE_PATH "/test/resources/description/test_generic_connection_description.yaml"

using tuw_ros_control_generic_test::FileLoader;
using tuw_ros_control_generic::GenericConnectionDescription;

class GenericConnectionDescriptionTest : public ::testing::Test
{
protected:
  GenericConnectionDescription generic_connection_description_ =
          GenericConnectionDescription(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));
};

TEST_F(GenericConnectionDescriptionTest, verifyHash)
{
  ASSERT_EQ(generic_connection_description_.getHash(), "/dev/ttyUSB0_57600");
}

TEST_F(GenericConnectionDescriptionTest, verifyPort)
{
  ASSERT_EQ(generic_connection_description_.getPort(), "/dev/ttyUSB0");
}

TEST_F(GenericConnectionDescriptionTest, verifyBaudrate)
{
  ASSERT_EQ(generic_connection_description_.getBaudrate(), 57600);
}
