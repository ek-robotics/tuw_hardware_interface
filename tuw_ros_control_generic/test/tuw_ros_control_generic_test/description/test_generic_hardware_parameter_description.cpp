// Copyright 2022 Eugen Kaltenegger

#include <gtest/gtest.h>

#include <tuw_ros_control_generic/description/generic_hardware_parameter_description.h>

using tuw_ros_control_generic::GenericHardwareParameterDescription;

TEST(TestGenericHardwareParameterDescription, verifyConstructorFromYaml_Target)
{
  YAML::Node yaml = YAML::Load("{identifier: ghp, address: 1, length: 1, range: { min: -1, max: 1 } }");
  GenericHardwareParameterDescription ghpd(yaml);

  ASSERT_TRUE(ghpd.getIdentifier());
  ASSERT_TRUE(ghpd.getAddress());
  ASSERT_TRUE(ghpd.getLength());
  ASSERT_TRUE(ghpd.getRange());

  ASSERT_FALSE(ghpd.getEnum());

  ASSERT_EQ(*ghpd.getIdentifier(), "ghp");
  ASSERT_EQ(*ghpd.getAddress(), 1);
  ASSERT_EQ(*ghpd.getLength(), 1);
  ASSERT_EQ(ghpd.getRange()->at("min"), -1);
  ASSERT_EQ(ghpd.getRange()->at("max"),  1);
}

TEST(TestGenericHardwareParameterDescription, verifyConstructorFromYaml_Actual)
{
  YAML::Node yaml = YAML::Load("{identifier: ghp, address: 1, length: 1}");
  GenericHardwareParameterDescription ghpd(yaml);

  ASSERT_TRUE(ghpd.getIdentifier());
  ASSERT_TRUE(ghpd.getAddress());
  ASSERT_TRUE(ghpd.getLength());

  ASSERT_FALSE(ghpd.getRange());
  ASSERT_FALSE(ghpd.getEnum());

  ASSERT_EQ(*ghpd.getIdentifier(), "ghp");
  ASSERT_EQ(*ghpd.getAddress(), 1);
  ASSERT_EQ(*ghpd.getLength(), 1);
}

TEST(TestGenericHardwareParameterDescription, verifyConstructorFromYaml_Enum)
{
  YAML::Node yaml = YAML::Load(
          "{identifier: ghp, description: \"a description\", address: 1, length: 1, enum: { a: 0, b: 1, c: 2 } }");
  GenericHardwareParameterDescription ghpd(yaml);

  ASSERT_TRUE(ghpd.getIdentifier());
  ASSERT_TRUE(ghpd.getAddress());
  ASSERT_TRUE(ghpd.getLength());
  ASSERT_TRUE(ghpd.getEnum());

  ASSERT_FALSE(ghpd.getRange());

  ASSERT_EQ(*ghpd.getIdentifier(), "ghp");
  ASSERT_EQ(*ghpd.getDescription(), "a description");
  ASSERT_EQ(*ghpd.getAddress(), 1);
  ASSERT_EQ(*ghpd.getLength(), 1);
  ASSERT_EQ(ghpd.getEnum()->at("a"), 0);
  ASSERT_EQ(ghpd.getEnum()->at("b"), 1);
  ASSERT_EQ(ghpd.getEnum()->at("c"), 2);
}

TEST(TestGenericHardwareParameterDescription, verifyConstructorFromYaml_Range)
{
  std::string yaml_string = "{identifier: ghp, description: \"a description\", address: 1, length: 1, range: { min: -1, max: 1 } }";
  YAML::Node yaml = YAML::Load(yaml_string);
  GenericHardwareParameterDescription ghpd(yaml);

  ASSERT_TRUE(ghpd.getIdentifier());
  ASSERT_TRUE(ghpd.getAddress());
  ASSERT_TRUE(ghpd.getLength());
  ASSERT_TRUE(ghpd.getRange());

  ASSERT_FALSE(ghpd.getEnum());

  ASSERT_EQ(*ghpd.getIdentifier(), "ghp");
  ASSERT_EQ(*ghpd.getDescription(), "a description");
  ASSERT_EQ(*ghpd.getAddress(), 1);
  ASSERT_EQ(*ghpd.getLength(), 1);
  ASSERT_EQ(ghpd.getRange()->at("min"), -1);
  ASSERT_EQ(ghpd.getRange()->at("max"),  1);
}
