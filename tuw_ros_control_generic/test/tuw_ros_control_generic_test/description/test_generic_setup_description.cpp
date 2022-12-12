// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include "../../tuw_ros_control_generic_test_util/include/file_loader.h"

#include <tuw_ros_control_generic/description/generic_setup_description.h>

#define TEST_FILE_PATH "/test/resources/description/test_generic_setup_description.yaml"

using tuw_ros_control_generic_test::FileLoader;
using tuw_ros_control_generic::GenericSetupDescription;

TEST(TestGenericSetupDescription, verifyConstructorFromYaml)
{
  GenericSetupDescription gsd(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_EQ(gsd.getJoints().size(), 3);
}