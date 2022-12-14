// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "../tuw_ros_control_generic_test_util/include/file_loader.h"

#include <tuw_ros_control_generic/description/generic_hardware_description.h>
#include <tuw_ros_control_generic/generic_config.h>
#include <tuw_ros_control_generic/generic_hardware.h>
#include <tuw_ros_control_generic/generic_joint.h>
#include <tuw_ros_control_generic/generic_setup_prefix.h>

#define TEST_FILE_PATH "/test/resources/test_generic_config.yaml"

using tuw_ros_control_generic_test::FileLoader;
using tuw_ros_control_generic::GenericConfig;
using tuw_ros_control_generic::GenericConfigDescription;
using tuw_ros_control_generic::GenericHardware;
using tuw_ros_control_generic::GenericHardwareDescription;
using tuw_ros_control_generic::GenericHardwareParameter;
using tuw_ros_control_generic::GenericHardwareParameterDescription;
using tuw_ros_control_generic::GenericJoint;
using tuw_ros_control_generic::GenericSetupPrefix;

class JointMock : public GenericJoint
{
public:
  JointMock() = default;
  MOCK_METHOD(std::string, getName, (), (override));
  MOCK_METHOD(void, write, (GenericHardwareParameter hardware_parameter, int* data), (override));
  MOCK_METHOD(void, read, (GenericHardwareParameter hardware_parameter, int* data), (override));
};

class GenericConfigTest : public ::testing::Test
{
protected:
  void SetUp() override {
    GenericSetupPrefix::setSetupName("test_setup");
    EXPECT_CALL(*mock_joint_, getName()).WillRepeatedly(testing::Return("test_joint"));
  }

  std::shared_ptr<JointMock> mock_joint_ = std::make_shared<JointMock>();
  std::shared_ptr<GenericHardware> generic_hardware_ =
          GenericHardware::getHardware(
                  GenericHardwareDescription(FileLoader::loadYAMLFromFile(TEST_FILE_PATH)["hardware"]));
  GenericConfigDescription generic_config_description_ =
          GenericConfigDescription(FileLoader::loadYAMLFromFile(TEST_FILE_PATH)["config"]);
};

TEST_F(GenericConfigTest, verifyConstructorWithoutConfig)
{
  EXPECT_CALL(*mock_joint_, read(generic_hardware_->getConfigIdentifierToParameter()->at("ecp"), testing::_)).Times(1);
  EXPECT_CALL(*mock_joint_, read(generic_hardware_->getConfigIdentifierToParameter()->at("rcp"), testing::_)).Times(1);

  GenericConfig generic_config = GenericConfig(mock_joint_, generic_hardware_);
}

TEST_F(GenericConfigTest, verifyServiceWithoutConfig)
{
  EXPECT_CALL(*mock_joint_, read(generic_hardware_->getConfigIdentifierToParameter()->at("ecp"), testing::_)).Times(1);
  EXPECT_CALL(*mock_joint_, read(generic_hardware_->getConfigIdentifierToParameter()->at("rcp"), testing::_)).Times(1);

  GenericConfig generic_config = GenericConfig(mock_joint_, generic_hardware_);

  EXPECT_TRUE(ros::service::waitForService("tuw_ros_control_generic_unit_test/test_joint/set_parameters", true));
  ros::Duration(0.01).sleep();  // required to supress warning about closed service
}

TEST_F(GenericConfigTest, verifyConstructorWithConfig)
{
  EXPECT_CALL(*mock_joint_, write(generic_hardware_->getConfigIdentifierToParameter()->at("ecp"), testing::_)).Times(1);
  EXPECT_CALL(*mock_joint_, write(generic_hardware_->getConfigIdentifierToParameter()->at("rcp"), testing::_)).Times(1);
  EXPECT_CALL(*mock_joint_, read(generic_hardware_->getConfigIdentifierToParameter()->at("ecp"), testing::_)).Times(2);
  EXPECT_CALL(*mock_joint_, read(generic_hardware_->getConfigIdentifierToParameter()->at("rcp"), testing::_)).Times(2);

  ROS_INFO("THE WARNINGS BELOW CAN BE IGNORED IN TESTS!");

  GenericConfig generic_config = GenericConfig(mock_joint_, generic_hardware_, generic_config_description_);
}

TEST_F(GenericConfigTest, verifyServiceWithConfig)
{
  EXPECT_CALL(*mock_joint_, write(generic_hardware_->getConfigIdentifierToParameter()->at("ecp"), testing::_)).Times(1);
  EXPECT_CALL(*mock_joint_, write(generic_hardware_->getConfigIdentifierToParameter()->at("rcp"), testing::_)).Times(1);
  EXPECT_CALL(*mock_joint_, read(generic_hardware_->getConfigIdentifierToParameter()->at("ecp"), testing::_)).Times(2);
  EXPECT_CALL(*mock_joint_, read(generic_hardware_->getConfigIdentifierToParameter()->at("rcp"), testing::_)).Times(2);

  ROS_INFO("THE WARNINGS BELOW CAN BE IGNORED IN TESTS!");

  GenericConfig generic_config = GenericConfig(mock_joint_, generic_hardware_, generic_config_description_);

  EXPECT_TRUE(ros::service::exists("tuw_ros_control_generic_unit_test/test_joint/set_parameters", false));
  ros::Duration(0.01).sleep();  // required to supress warning about closed service
}
