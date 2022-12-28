// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <tuw_hardware_interface_template/utils/file_loader.h>

#include <tuw_hardware_interface_template/description/generic_connection_description.h>
#include <tuw_hardware_interface_template/generic_connection.h>
#include <tuw_hardware_interface_template/generic_setup_prefix.h>

using tuw_hardware_interface::FileLoader;
using tuw_ros_control_generic::GenericConnection;
using tuw_ros_control_generic::GenericConnectionDescription;
using tuw_ros_control_generic::GenericSetupPrefix;

#define TEST_FILE_PATH "/test/resources/generic_connection_test.yaml"

class GenericConnectionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    GenericSetupPrefix::setSetupName("test_setup");
  }

  std::shared_ptr<GenericConnectionDescription> generic_connection_description_ =
          std::make_shared<GenericConnectionDescription>(FileLoader::loadYAMLFromFile(TEST_FILE_PATH));
};

TEST_F(GenericConnectionTest, verifyPointer)
{
  ASSERT_EQ(GenericConnection::getConnection(generic_connection_description_).get(), nullptr);
}
