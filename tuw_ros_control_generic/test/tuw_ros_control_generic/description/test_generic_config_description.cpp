// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include "../tool/file_loader.cpp"

#include <tuw_ros_control_generic/description/generic_config_description.h>

#define TEST_FILE_PATH "/test/resources/description/test_generic_config_description.yaml"

using tuw_ros_control_generic::GenericConfigDescription;

TEST(TestGenericConfigDescription, verifyElements)
{
  GenericConfigDescription gcd(loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_EQ(gcd.getConfigMap().at("config_value_0"), 0);
  ASSERT_EQ(gcd.getConfigMap().at("config_value_1"), 1);
  ASSERT_EQ(gcd.getConfigMap().at("config_value_2"), 2);
}

TEST(TestGenericConfigDescription, verifyLength)
{
  GenericConfigDescription gcd(loadYAMLFromFile(TEST_FILE_PATH));

  ASSERT_EQ(gcd.getConfigMap().size(), 3);
}