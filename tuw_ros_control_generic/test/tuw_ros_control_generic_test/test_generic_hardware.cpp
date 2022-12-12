// Copyright 2022 Eugen Kaltenegger

#include <tuw_ros_control_generic/description/generic_hardware_description.h>
#include <tuw_ros_control_generic/generic_hardware.h>

#include <string>

#include <ros/package.h>

#include <gtest/gtest.h>

using tuw_ros_control_generic::GenericHardware;
using tuw_ros_control_generic::GenericHardwareDescription;

#define TEST_FILE_PATH "/test/resources/test_generic_hardware.yaml"

TEST(TestGenericHardware, verifyTargetMode)
{
  std::string path = ros::package::getPath("tuw_ros_control_generic") + std::string(TEST_FILE_PATH);
  YAML::Node yaml = YAML::LoadFile(path);
  GenericHardware gh = GenericHardware(GenericHardwareDescription(yaml));

  ASSERT_TRUE(gh.supportsTargetMode(GenericHardware::Mode::POSITION));
  ASSERT_TRUE(gh.supportsTargetMode(GenericHardware::Mode::VELOCITY));
  ASSERT_TRUE(gh.supportsTargetMode(GenericHardware::Mode::EFFORT));

  ASSERT_EQ(*gh.getTargetParameterForMode(GenericHardware::Mode::POSITION).getIdentifier(), "target_position");
  ASSERT_EQ(*gh.getTargetParameterForMode(GenericHardware::Mode::VELOCITY).getIdentifier(), "target_velocity");
  ASSERT_EQ(*gh.getTargetParameterForMode(GenericHardware::Mode::EFFORT).getIdentifier(), "target_effort");
}

TEST(TestGenericHardware, verifyActualMode)
{
  std::string path = ros::package::getPath("tuw_ros_control_generic") + std::string(TEST_FILE_PATH);
  YAML::Node yaml = YAML::LoadFile(path);
  GenericHardware gh = GenericHardware(GenericHardwareDescription(yaml));

  ASSERT_TRUE(gh.supportsActualMode(GenericHardware::Mode::POSITION));
  ASSERT_TRUE(gh.supportsActualMode(GenericHardware::Mode::VELOCITY));
  ASSERT_TRUE(gh.supportsActualMode(GenericHardware::Mode::EFFORT));

  ASSERT_EQ(*gh.getActualParameterForMode(GenericHardware::Mode::POSITION).getIdentifier(), "actual_position");
  ASSERT_EQ(*gh.getActualParameterForMode(GenericHardware::Mode::VELOCITY).getIdentifier(), "actual_velocity");
  ASSERT_EQ(*gh.getActualParameterForMode(GenericHardware::Mode::EFFORT).getIdentifier(), "actual_effort");
}

TEST(TestGenericHardware, verifyConfigParameters)
{
  std::string path = ros::package::getPath("tuw_ros_control_generic") + std::string(TEST_FILE_PATH);
  YAML::Node yaml = YAML::LoadFile(path);
  GenericHardware gh = GenericHardware(GenericHardwareDescription(yaml));

  ASSERT_TRUE(gh.getConfigIdentifierToParameter());

  ASSERT_EQ(*gh.getConfigIdentifierToParameter()->at("ecp").getIdentifier(), "ecp");
  ASSERT_EQ(*gh.getConfigIdentifierToParameter()->at("rcp").getIdentifier(), "rcp");
}
