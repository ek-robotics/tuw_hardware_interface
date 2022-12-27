// Copyright 2022 Eugen Kaltenegger

#include "tuw_dynamixel_hardware_interface/dynamixel_hardware_interface.h"
#include "tuw_dynamixel_hardware_interface/dynamixel_connection.h"
#include "tuw_dynamixel_hardware_interface/description/dynamixel_setup_description.h"

#include "tuw_ros_control_generic/description/generic_joint_description.h"
#include "tuw_ros_control_generic/description/generic_setup_description.h"
#include "tuw_ros_control_generic/generic_config.h"
#include "tuw_ros_control_generic/generic_setup_prefix.h"

using tuw_hardware_interface::DynamixelHardwareInterface;
using tuw_hardware_interface::DynamixelConnection;
using tuw_ros_control_generic::GenericHardware;
using tuw_ros_control_generic::GenericConfig;
using tuw_ros_control_generic::GenericHardwareDescription;
using tuw_ros_control_generic::GenericConfigDescription;
using tuw_ros_control_generic::GenericSetupPrefix;
using tuw_ros_control_generic::GenericSetupDescription;

std::string GenericHardwareInterface::setup_parameter_ = "dynamixel_hardware_interface_setup";

bool tuw_hardware_interface::DynamixelHardwareInterface::init(ros::NodeHandle& basic_node_handle,
                                                              ros::NodeHandle& hardware_node_handle)
{
  std::string setup_file_path;
  YAML::Node setup_yaml;

  if (!basic_node_handle.hasParam(GenericHardwareInterface::setup_parameter_))
  {
    ROS_ERROR("[%s] parameter %s is not set", PREFIX, GenericHardwareInterface::setup_parameter_.LOG);
    return false;
  }

  try
  {
    basic_node_handle.getParam(GenericHardwareInterface::setup_parameter_, setup_file_path);
  }
  catch(...)
  {
    ROS_ERROR("[%s] PARAMETER FOR SETUP FILE NOT FOUND (\"%s\")", PREFIX, GenericHardwareInterface::setup_parameter_.LOG);
    return false;
  }

  try
  {
    setup_yaml = YAML::LoadFile(setup_file_path);
  }
  catch (...)
  {
    ROS_ERROR("[%s] SETUP FILE NOT FOUND OR INVALID (\"%s\")", PREFIX, setup_file_path.LOG);
    return false;
  }

  try
  {
    DynamixelSetupDescription setup_description = DynamixelSetupDescription(setup_yaml);
    GenericSetupPrefix::setSetupName(setup_description.getName());
    this->initJoints(setup_description.getDynamixelJoints());
    return true;
  }
  catch (std::exception &exception)
  {
    ROS_ERROR("[%s] ERROR INITIALIZING NODE: %s", PREFIX, exception.what());
    return false;
  }
}

bool DynamixelHardwareInterface::initJoints(std::list<DynamixelJointDescription> joint_descriptions)
{
  try
  {
    for (const auto& joint_description : joint_descriptions)
    {
      this->initJoint(joint_description);
    }

    registerInterface(&this->joint_state_interface);
    registerInterface(&this->joint_position_interface);
    registerInterface(&this->joint_velocity_interface);
    registerInterface(&this->joint_effort_interface);

    ROS_INFO("[%s] FINISH setting up %zu joint(s)", PREFIX, this->joints_.size());

    return true;
  }
  catch(...)
  {
    return false;
  }
}

bool DynamixelHardwareInterface::initJoint(DynamixelJointDescription joint_description)
{
  ROS_INFO("CALLING DYNAMIXEL INIT JOINT");
  try
  {
    std::shared_ptr<GenericJoint> joint = std::make_shared<GenericJoint>(joint_description);

    std::shared_ptr<DynamixelConnectionDescription> connection_description = joint_description.getDynamixelConnectionDescription();
    std::shared_ptr<GenericHardwareDescription> hardware_description = joint_description.getHardwareDescription();
    std::shared_ptr<GenericConfigDescription> config_description = joint_description.getConfigDescription();

    ROS_INFO("A");
    std::shared_ptr<DynamixelConnection> connection =
            DynamixelConnection::getConnection(connection_description);
    joint->setConnection(connection);


    ROS_INFO("B");
    std::shared_ptr<GenericHardware> hardware =
            GenericHardware::getHardware(*hardware_description);
    joint->setHardware(hardware);



    ROS_INFO("C");
//    std::shared_ptr<GenericConfig> config =
//            std::make_shared<GenericConfig>(joint, hardware);
//            std::make_shared<GenericConfig>(joint, hardware, *config_description);
    ROS_INFO("D");
//    joint->setConfig(config);
    ROS_INFO("E");
    // create connection and set it to joint
    // create hardware and set it to joint
    // create config and set it to joint

    this->joints_.push_back(joint);

    this->joint_state_interface.registerHandle(*joint->getJointStateHandle());
    this->joint_position_interface.registerHandle(*joint->getJointPositionHandle());
    this->joint_velocity_interface.registerHandle(*joint->getJointVelocityHandle());
    this->joint_effort_interface.registerHandle(*joint->getJointEffortHandle());

    ROS_INFO("[%s] FINISH setting up joint %s", PREFIX, joint->getName().LOG);

    return true;
  }
  catch(...)
  {
    return false;
  }
}
