// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include "../tool/file_loader.cpp"

#include <tuw_ros_control_generic/description/generic_setup_description.h>

#define TEST_FILE_PATH "/test/resources/description/test_generic_setup_description.yaml"

using tuw_ros_control_generic::GenericSetupDescription;

TEST(TestGenericSetupDescription, verifyConstructorFromYaml)
{
  GenericSetupDescription gsd(loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_EQ(gsd.getJoints().size(), 3);
}