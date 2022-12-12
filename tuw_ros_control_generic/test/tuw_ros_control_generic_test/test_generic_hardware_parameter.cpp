// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <tuw_ros_control_generic/description/generic_hardware_parameter_description.h>
#include <tuw_ros_control_generic/generic_hardware_parameter.h>

using tuw_ros_control_generic::GenericHardwareParameter;
using tuw_ros_control_generic::GenericHardwareParameterDescription;

TEST(TestGenericHardwareParameter, verifyConstructor_Target)
{
  YAML::Node yaml = YAML::Load("{identifier: ghp, address: 1, length: 1, range: { min: -1, max: 1 } }");
  GenericHardwareParameter ghp = GenericHardwareParameter(GenericHardwareParameterDescription(yaml));

  ASSERT_TRUE(ghp.getIdentifier());
  ASSERT_TRUE(ghp.getAddress());
  ASSERT_TRUE(ghp.getLength());
  ASSERT_TRUE(ghp.getRange());

  ASSERT_FALSE(ghp.getEnum());

  ASSERT_EQ(*ghp.getIdentifier(), "ghp");
  ASSERT_EQ(*ghp.getAddress(), 1);
  ASSERT_EQ(*ghp.getLength(), 1);
  ASSERT_EQ(ghp.getRange()->at("min"), -1);
  ASSERT_EQ(ghp.getRange()->at("max"),  1);
}

TEST(TestGenericHardwareParameter, verifyConstructor_Actual)
{
  YAML::Node yaml = YAML::Load("{identifier: ghp, address: 1, length: 1}");
  GenericHardwareParameter ghp = GenericHardwareParameter(GenericHardwareParameterDescription(yaml));

  ASSERT_TRUE(ghp.getIdentifier());
  ASSERT_TRUE(ghp.getAddress());
  ASSERT_TRUE(ghp.getLength());

  ASSERT_FALSE(ghp.getRange());
  ASSERT_FALSE(ghp.getEnum());

  ASSERT_EQ(*ghp.getIdentifier(), "ghp");
  ASSERT_EQ(*ghp.getAddress(), 1);
  ASSERT_EQ(*ghp.getLength(), 1);
}

TEST(TestGenericHardwareParameter, verifyConstructor_Enum)
{
  YAML::Node yaml = YAML::Load(
          "{identifier: ghp, description: \"a description\", address: 1, length: 1, enum: { a: 0, b: 1, c: 2 } }");
  GenericHardwareParameter ghp = GenericHardwareParameter(GenericHardwareParameterDescription(yaml));

  ASSERT_TRUE(ghp.getIdentifier());
  ASSERT_TRUE(ghp.getAddress());
  ASSERT_TRUE(ghp.getLength());
  ASSERT_TRUE(ghp.getEnum());

  ASSERT_FALSE(ghp.getRange());

  ASSERT_EQ(*ghp.getIdentifier(), "ghp");
  ASSERT_EQ(*ghp.getDescription(), "a description");
  ASSERT_EQ(*ghp.getAddress(), 1);
  ASSERT_EQ(*ghp.getLength(), 1);
  ASSERT_EQ(ghp.getEnum()->at("a"), 0);
  ASSERT_EQ(ghp.getEnum()->at("b"), 1);
  ASSERT_EQ(ghp.getEnum()->at("c"), 2);
}

TEST(TestGenericHardwareParameter, verifyConstructor_Range)
{
  YAML::Node yaml = YAML::Load(
          "{identifier: ghp, description: \"a description\", address: 1, length: 1, range: { min: -1, max: 1 } }");
  GenericHardwareParameter ghp = GenericHardwareParameter(GenericHardwareParameterDescription(yaml));

  ASSERT_TRUE(ghp.getIdentifier());
  ASSERT_TRUE(ghp.getAddress());
  ASSERT_TRUE(ghp.getLength());
  ASSERT_TRUE(ghp.getRange());

  ASSERT_FALSE(ghp.getEnum());

  ASSERT_EQ(*ghp.getIdentifier(), "ghp");
  ASSERT_EQ(*ghp.getDescription(), "a description");
  ASSERT_EQ(*ghp.getAddress(), 1);
  ASSERT_EQ(*ghp.getLength(), 1);
  ASSERT_EQ(ghp.getRange()->at("min"), -1);
  ASSERT_EQ(ghp.getRange()->at("max"),  1);
}

TEST(TestGenericHardwareParameter, verifyType_Target)
{
  YAML::Node yaml = YAML::Load("{identifier: ghp, address: 1, length: 1, range: { min: -1, max: 1 } }");
  GenericHardwareParameter ghp = GenericHardwareParameter(GenericHardwareParameterDescription(yaml));

  ASSERT_EQ(ghp.getType(), GenericHardwareParameter::Type::TARGET);
}

TEST(TestGenericHardwareParameter, verifyType_Actual)
{
  YAML::Node yaml = YAML::Load("{identifier: ghp, address: 1, length: 1}");
  GenericHardwareParameter ghp = GenericHardwareParameter(GenericHardwareParameterDescription(yaml));

  ASSERT_EQ(ghp.getType(), GenericHardwareParameter::Type::ACTUAL);
}

TEST(TestGenericHardwareParameter, verifyType_Enum)
{
  YAML::Node yaml = YAML::Load(
          "{identifier: ghp, description: \"a description\", address: 1, length: 1, enum: { a: 0, b: 1, c: 2 } }");
  GenericHardwareParameter ghp = GenericHardwareParameter(GenericHardwareParameterDescription(yaml));

  ASSERT_EQ(ghp.getType(), GenericHardwareParameter::Type::ENUM);
}

TEST(TestGenericHardwareParameter, verifyType_Range)
{
  YAML::Node yaml = YAML::Load(
          "{identifier: ghp, description: \"a description\", address: 1, length: 1, range: { min: -1, max: 1 } }");
  GenericHardwareParameter ghp = GenericHardwareParameter(GenericHardwareParameterDescription(yaml));

  ASSERT_EQ(ghp.getType(), GenericHardwareParameter::Type::RANGE);
}
