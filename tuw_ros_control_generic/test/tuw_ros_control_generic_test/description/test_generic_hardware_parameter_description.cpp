// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <tuw_ros_control_generic/description/generic_hardware_parameter_description.h>

using tuw_ros_control_generic::GenericHardwareParameterDescription;

class GenericHardwareParameterDescriptionTest : public ::testing::Test
{
protected:
  GenericHardwareParameterDescription target_ = GenericHardwareParameterDescription(YAML::Load(
          "{identifier: ghp,"
          " address: 1,"
          " length: 1,"
          " range: { min: -1, max: 1 } }"));
  GenericHardwareParameterDescription actual_ = GenericHardwareParameterDescription(YAML::Load(
          "{identifier: ghp,"
          " address: 1,"
          " length: 1}"));

  GenericHardwareParameterDescription enum_ = GenericHardwareParameterDescription(YAML::Load(
          "{identifier: ghp,"
          " description: \"a description\", "
          " address: 1,"
          " length: 1,"
          " enum: { a: 0, b: 1, c: 2 } }"));

  GenericHardwareParameterDescription range_ = GenericHardwareParameterDescription(YAML::Load(
          "{identifier: ghp,"
          " description: \"a description\","
          " address: 1,"
          " length: 1,"
          " range: { min: -1, max: 1 } }"));
};

TEST_F(GenericHardwareParameterDescriptionTest, verifyTargetPointers)
{
  ASSERT_TRUE(target_.getIdentifier());
  ASSERT_TRUE(target_.getAddress());
  ASSERT_TRUE(target_.getLength());
  ASSERT_TRUE(target_.getRange());
  ASSERT_FALSE(target_.getEnum());
}

TEST_F(GenericHardwareParameterDescriptionTest, verifyTargetValues)
{
  ASSERT_EQ(*target_.getIdentifier(), "ghp");
  ASSERT_EQ(*target_.getAddress(), 1);
  ASSERT_EQ(*target_.getLength(), 1);
  ASSERT_EQ(target_.getRange()->at("min"), -1);
  ASSERT_EQ(target_.getRange()->at("max"), 1);
}

TEST_F(GenericHardwareParameterDescriptionTest, verifyActualPointers)
{
  ASSERT_TRUE(actual_.getIdentifier());
  ASSERT_TRUE(actual_.getAddress());
  ASSERT_TRUE(actual_.getLength());
  ASSERT_FALSE(actual_.getRange());
  ASSERT_FALSE(actual_.getEnum());
}

TEST_F(GenericHardwareParameterDescriptionTest, verifyActualValues)
{
  ASSERT_EQ(*actual_.getIdentifier(), "ghp");
  ASSERT_EQ(*actual_.getAddress(), 1);
  ASSERT_EQ(*actual_.getLength(), 1);
}

TEST_F(GenericHardwareParameterDescriptionTest, verifyEnumPointers)
{
  ASSERT_TRUE(enum_.getIdentifier());
  ASSERT_TRUE(enum_.getAddress());
  ASSERT_TRUE(enum_.getLength());
  ASSERT_TRUE(enum_.getEnum());
  ASSERT_FALSE(enum_.getRange());
}

TEST_F(GenericHardwareParameterDescriptionTest, verifyEnumValues)
{
  ASSERT_EQ(*enum_.getIdentifier(), "ghp");
  ASSERT_EQ(*enum_.getDescription(), "a description");
  ASSERT_EQ(*enum_.getAddress(), 1);
  ASSERT_EQ(*enum_.getLength(), 1);
  ASSERT_EQ(enum_.getEnum()->at("a"), 0);
  ASSERT_EQ(enum_.getEnum()->at("b"), 1);
  ASSERT_EQ(enum_.getEnum()->at("c"), 2);
}

TEST_F(GenericHardwareParameterDescriptionTest, verifyRangePointers)
{
  ASSERT_TRUE(range_.getIdentifier());
  ASSERT_TRUE(range_.getAddress());
  ASSERT_TRUE(range_.getLength());
  ASSERT_TRUE(range_.getRange());
  ASSERT_FALSE(range_.getEnum());
}

TEST_F(GenericHardwareParameterDescriptionTest, verifyRangeValues)
{
  ASSERT_EQ(*range_.getIdentifier(), "ghp");
  ASSERT_EQ(*range_.getDescription(), "a description");
  ASSERT_EQ(*range_.getAddress(), 1);
  ASSERT_EQ(*range_.getLength(), 1);
  ASSERT_EQ(range_.getRange()->at("min"), -1);
  ASSERT_EQ(range_.getRange()->at("max"), 1);
}
