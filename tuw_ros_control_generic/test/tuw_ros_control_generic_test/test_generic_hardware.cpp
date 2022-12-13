// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <tuw_ros_control_generic/generic_hardware.h>

#include "../tuw_ros_control_generic_test_util/include/file_loader.h"

using tuw_ros_control_generic_test::FileLoader;
using tuw_ros_control_generic::GenericHardware;
using tuw_ros_control_generic::GenericHardwareDescription;

#define TEST_FILE_PATH "/test/resources/test_generic_hardware.yaml"

class GenericHardwareTest : public ::testing::Test
{
protected:
  GenericHardware generic_hardware_ =
          GenericHardware(GenericHardwareDescription(FileLoader::loadYAMLFromFile(TEST_FILE_PATH)));
};

TEST_F(GenericHardwareTest, verifyName)
{
  ASSERT_EQ(generic_hardware_.getName(), "generic_hardware");
}

TEST_F(GenericHardwareTest, verifyTargetModes)
{
  ASSERT_TRUE(generic_hardware_.supportsTargetMode(GenericHardware::Mode::POSITION));
  ASSERT_TRUE(generic_hardware_.supportsTargetMode(GenericHardware::Mode::VELOCITY));
  ASSERT_TRUE(generic_hardware_.supportsTargetMode(GenericHardware::Mode::EFFORT));
}

TEST_F(GenericHardwareTest, verifyTargetIdentifiers)
{
  ASSERT_EQ(*generic_hardware_.getTargetParameterForMode(GenericHardware::Mode::POSITION).getIdentifier(), "target_position");
  ASSERT_EQ(*generic_hardware_.getTargetParameterForMode(GenericHardware::Mode::VELOCITY).getIdentifier(), "target_velocity");
  ASSERT_EQ(*generic_hardware_.getTargetParameterForMode(GenericHardware::Mode::EFFORT).getIdentifier(), "target_effort");
}

TEST_F(GenericHardwareTest, verifyActualModes)
{
  ASSERT_TRUE(generic_hardware_.supportsActualMode(GenericHardware::Mode::POSITION));
  ASSERT_TRUE(generic_hardware_.supportsActualMode(GenericHardware::Mode::VELOCITY));
  ASSERT_TRUE(generic_hardware_.supportsActualMode(GenericHardware::Mode::EFFORT));
}

TEST_F(GenericHardwareTest, verifyActualIdentifiers)
{
  ASSERT_EQ(*generic_hardware_.getActualParameterForMode(GenericHardware::Mode::POSITION).getIdentifier(), "actual_position");
  ASSERT_EQ(*generic_hardware_.getActualParameterForMode(GenericHardware::Mode::VELOCITY).getIdentifier(), "actual_velocity");
  ASSERT_EQ(*generic_hardware_.getActualParameterForMode(GenericHardware::Mode::EFFORT).getIdentifier(), "actual_effort");
}

TEST_F(GenericHardwareTest, verifyConfigParametersPointer)
{
  ASSERT_TRUE(generic_hardware_.getConfigIdentifiers());
}

TEST_F(GenericHardwareTest, verifyConfigParametersSize)
{
  ASSERT_EQ(generic_hardware_.getConfigIdentifiers()->size(), 2);
}

TEST_F(GenericHardwareTest, verifyConfigIdentifierToParameterPointer)
{
  ASSERT_TRUE(generic_hardware_.getConfigIdentifierToParameter());
}

TEST_F(GenericHardwareTest, verifyConfigIdentifierToParameterValues)
{
  ASSERT_EQ(*generic_hardware_.getConfigIdentifierToParameter()->at("ecp").getIdentifier(), "ecp");
  ASSERT_EQ(*generic_hardware_.getConfigIdentifierToParameter()->at("rcp").getIdentifier(), "rcp");
}
