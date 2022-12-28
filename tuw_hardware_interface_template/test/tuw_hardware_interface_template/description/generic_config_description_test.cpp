// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <tuw_hardware_interface_template/utils/file_loader.h>

#include <tuw_hardware_interface_template/description/generic_config_description.h>

#define TEST_FILE_PATH "/test/resources/description/generic_config_description_test.yaml"

using tuw_hardware_interface::FileLoader;
using tuw_ros_control_generic::GenericConfigDescription;

class GenericConfigDescriptionTest : public ::testing::Test
{
protected:
  GenericConfigDescription generic_connection_description_ =
          GenericConfigDescription(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));
};

TEST_F(GenericConfigDescriptionTest, verifyElements)
{
  ASSERT_EQ(generic_connection_description_.getConfigMap().at("config_value_0"), 0);
  ASSERT_EQ(generic_connection_description_.getConfigMap().at("config_value_1"), 1);
  ASSERT_EQ(generic_connection_description_.getConfigMap().at("config_value_2"), 2);
}

TEST_F(GenericConfigDescriptionTest, verifyLength)
{
  ASSERT_EQ(generic_connection_description_.getConfigMap().size(), 3);
}
