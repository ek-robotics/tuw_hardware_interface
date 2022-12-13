// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include "../../tuw_ros_control_generic_test_util/include/file_loader.h"

#include <tuw_ros_control_generic/description/generic_setup_description.h>

#define TEST_FILE_PATH "/test/resources/description/test_generic_setup_description.yaml"

using tuw_ros_control_generic_test::FileLoader;
using tuw_ros_control_generic::GenericSetupDescription;

class GenericSetupDescriptionTest : public ::testing::Test
{
protected:
  GenericSetupDescription generic_setup_description_ =
          GenericSetupDescription(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));
};

TEST_F(GenericSetupDescriptionTest, verifySetupName)
{
  ASSERT_EQ(generic_setup_description_.getSetupName(), "setup_name");
}

TEST_F(GenericSetupDescriptionTest, verifyJointsNumber)
{
  ASSERT_EQ(generic_setup_description_.getJoints().size(), 3);
}
