/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  George Kouros
*********************************************************************/

#include "motor_hardware_interface/four_wheel_steer_hardware_interface.h"

namespace pandora_hardware_interface
{
namespace motor
{
  FourWheelSteerHardwareInterface::FourWheelSteerHardwareInterface(
    ros::NodeHandle nodeHandle)
   :
    nodeHandle_(nodeHandle)
  {
    motorHandler_ = new SerialEposHandler("/dev/epos", 115200, 500);

    // load joint names from parameter server
    loadJointConfiguration();

    // connect and register the joint state interfaces
    for (int ii = 0; ii < motorJointNames_.size(); ii++)
    {
      motorPosition_[ii] = 0;
      motorVelocity_[ii] = 0;
      motorEffort_[ii] = 0;
      hardware_interface::JointStateHandle motorJointStateHandle(
        motorJointNames_[ii],
        &motorPosition_[ii],
        &motorVelocity_[ii],
        &motorEffort_[ii]);
      motorJointStateInterface_.registerHandle(motorJointStateHandle);
      registerInterface(&motorJointStateInterface_);
    }

    // connect and register the joint velocity interface for the motor/s
    for (int ii = 0; ii < motorJointNames_.size(); ii++)
    {
      hardware_interface::JointHandle motorJointVelocityHandle(
        motorJointStateInterface_.getHandle(motorJointNames_[ii]),
        &motorCommand_[ii]);
      motorVelocityJointInterface_.registerHandle(motorJointVelocityHandle);
      registerInterface(&motorVelocityJointInterface_);
    }

    // connect and register the joint position interfaces for the steer servos
    for (int ii = 0; ii < servoJointNames_.size(); ii++)
    {
      hardware_interface::JointHandle servoJointPositionHandle(
        motorJointStateInterface_.getHandle(servoJointNames_[ii]),
        &servoCommand_[ii]);
      servoPositionJointInterface_.registerHandle(servoJointPositionHandle);
      registerInterface(&servoPositionJointInterface_);

      ros::Publisher publisher =
        nodeHandle_.advertise<std_msgs::Float64>(servoCommandTopics_[ii], 1);
      steerServoPositionPublishers_.push_back(publisher);
    }
  }


  FourWheelSteerHardwareInterface::~FourWheelSteerHardwareInterface()
  {
    delete motorHandler_;
    delete motorNodeId_;
    delete motorRatio_;
    delete motorCommand_;
    delete motorPosition_;
    delete motorVelocity_;
    delete motorEffort_;
    delete motorMinVelocity_;
    delete motorMaxVelocity_;
    delete servoCommand_;
    delete servoPosition_;
    delete servoMinPosition_;
    delete servoMaxPosition_;
  }


  void FourWheelSteerHardwareInterface::read(const ros::Duration& period)
  {
    if (motorHandler_->readAllRpms() != motorJointNames_.size())
    {
      ROS_ERROR("[MOTORS] Failed to read velocity of all motors. Exiting...");
      exit(-1);
    }
    else
    {
      for (int ii = 0; ii < motorJointNames_.size(); ii++)
      {
        motorVelocity_[ii] =
          motorHandler_->getRpm(motorNodeId_[ii]) / motorRatio_[ii];
        motorPosition_[ii] =
          motorPosition_[ii] + period.toSec() * motorVelocity_[ii];
      }
    }
  }


  void FourWheelSteerHardwareInterface::write()
  {
    for (int ii = 0; ii < motorJointNames_.size(); ii++)
    {
      int32_t motorRpm = motorCommand_[ii] * motorRatio_[ii];
      motorHandler_->setRpmCommand(motorNodeId_[ii], motorRpm);
    }

    if (motorHandler_->writeAllRpms() != motorJointNames_.size())
    {
      ROS_ERROR("[MOTORS] Failed to write rpms to all motors. Exiting...");
      exit(-1);
    }

    for (int ii = 0; ii < servoJointNames_.size(); ii++)
    {
      std_msgs::Float64 msg;
      msg.data = servoCommand_[ii];
      steerServoPositionPublishers_[ii].publish(msg);
    }
  }


  void FourWheelSteerHardwareInterface::loadJointConfiguration()
  {
    XmlRpc::XmlRpcValue motorJointList;
    nodeHandle_.getParam("motor_joints", motorJointList);
    ROS_ASSERT(
      motorJointList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    motorNodeId_ = new uint16_t[motorJointList.size()];
    motorRatio_ = new double[motorJointList.size()];
    motorCommand_ = new double[motorJointList.size()];
    motorPosition_ = new double[motorJointList.size()];
    motorVelocity_ = new double[motorJointList.size()];
    motorEffort_ = new double[motorJointList.size()];
    motorMinVelocity_ = new double[motorJointList.size()];
    motorMaxVelocity_ = new double[motorJointList.size()];

    for (int ii = 0; ii < motorJointList.size(); ii++)
    {
      ROS_ASSERT(
        motorJointList[ii].getType() == XmlRpc::XmlRpcValue::TypeStruct);

      ROS_ASSERT(
        motorJointList[ii]["joint_name"].getType() ==
        XmlRpc::XmlRpcValue::TypeString);
      motorJointNames_.push_back(
        static_cast<std::string>(motorJointList[ii]["joint_name"]));

      ROS_ASSERT(
        motorJointList[ii]["node_id"].getType() ==
        XmlRpc::XmlRpcValue::TypeInt);
      motorNodeId_[ii] = static_cast<int>(motorJointList[ii]["node_id"]);

      ROS_ASSERT(
        motorJointList[ii]["ratio"].getType() ==
        XmlRpc::XmlRpcValue::TypeDouble);
      motorRatio_[ii] = static_cast<double>(motorJointList[ii]["ratio"]);

      ROS_ASSERT(
        motorJointList[ii]["min_velocity"].getType() ==
        XmlRpc::XmlRpcValue::TypeDouble);
      motorMinVelocity_[ii] =
        static_cast<double>(motorJointList[ii]["min_velocity"]);

      ROS_ASSERT(
        motorJointList[ii]["max_velocity"].getType() ==
        XmlRpc::XmlRpcValue::TypeDouble);
      motorMaxVelocity_[ii] =
        static_cast<double>(motorJointList[ii]["max_velocity"]);
    }

    XmlRpc::XmlRpcValue servoJointList;
    nodeHandle_.getParam("servo_joints", servoJointList);
    ROS_ASSERT(
      servoJointList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    servoCommand_ = new double[servoJointList.size()];
    servoPosition_ = new double[servoJointList.size()];
    servoMinPosition_ = new double[servoJointList.size()];
    servoMaxPosition_ = new double[servoJointList.size()];

    for (int ii = 0; ii < servoJointList.size(); ii++)
    {
      ROS_ASSERT(
        servoJointList[ii].getType() == XmlRpc::XmlRpcValue::TypeArray);

      ROS_ASSERT(
        servoJointList[ii]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
      servoJointNames_.push_back(
        static_cast<std::string>(servoJointList[ii]["name"]));

      ROS_ASSERT(
        servoJointList[ii]["command_topic"].getType() ==
        XmlRpc::XmlRpcValue::TypeString);
      servoCommandTopics_.push_back(
        static_cast<std::string>(servoJointList[ii]["command_topic"]));

      ROS_ASSERT(
        servoJointList[ii]["min_position"].getType() ==
        XmlRpc::XmlRpcValue::TypeDouble);
      servoMinPosition_[ii] =
        static_cast<double>(servoJointList[ii]["min_position"]);

      ROS_ASSERT(
        servoJointList[ii]["max_position"].getType() ==
        XmlRpc::XmlRpcValue::TypeDouble);
      servoMaxPosition_[ii] =
       static_cast<double>(servoJointList[ii]["max_position"]);
    }
  }

}  // namespace motor
}  // namespace pandora_hardware_interface
