/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
* Author:  Evangelos Apostolidis
* Author:  Elisavet Papadopoulou
* Author:  Kostantinos Zisis
*********************************************************************/
#include "motor_hardware_interface/motor_hardware_interface.h"

namespace pandora_hardware_interface
{
namespace motor
{
  MotorHardwareInterface::MotorHardwareInterface(
    ros::NodeHandle nodeHandle)
  :
    nodeHandle_(nodeHandle)
  {
    motors_ = new SerialEpos2Handler();

    nodeHandle_.getParam("max_RPM", maxRPM_);
    nodeHandle_.getParam("gearbox_ratio", gearboxRatio_);
    nodeHandle_.getParam("max_torque", maxTorque_);
    nodeHandle_.getParam("joints/robot_movement_joints", jointNames_);
    nodeHandle_.getParam("joints/joint_to_controller_mapping",
      jointToControllerMap_);

    // connect and register the joint state interface
    for (int ii = 0; ii < jointNames_.size(); ii++)
    {
      position_[ii] = 0;
      velocity_[ii] = 0;
      effort_[ii] = 0;
      hardware_interface::JointStateHandle jointStateHandle(
        jointNames_[ii],
        &position_[ii],
        &velocity_[ii],
        &effort_[ii]);
      jointStateInterface_.registerHandle(jointStateHandle);
    }
    registerInterface(&jointStateInterface_);

    // connect and register the joint velocity interface
    for (int ii = 0; ii < jointNames_.size(); ii++)
    {
      hardware_interface::JointHandle jointVelocityHandle(
        jointStateInterface_.getHandle(jointNames_[ii]),
        &velCommand_[ii]);
      velocityJointInterface_.registerHandle(jointVelocityHandle);
    }
    registerInterface(&velocityJointInterface_);

    // Add effortJointInterface!
    for (int ii = 0; ii < jointNames_.size(); ii++)
    {
      hardware_interface::JointHandle jointEffortHandle(
        jointStateInterface_.getHandle(jointNames_[ii]),
        &torqueCommand_[ii]);
      effortJointInterface_.registerHandle(jointEffortHandle);
    }
    registerInterface(&effortJointInterface_);

    // Set motor control mode to velocity control mode
    motors_->setMode(0);

    // Initiallize jointLimits
    for (int ii = 0; ii < jointNames_.size(); ii++)
    {
      hardware_interface::JointHandle jointLimitsHandle =
        velocityJointInterface_.getHandle(jointNames_[ii]);

      if (!joint_limits_interface::getJointLimits(jointNames_[ii], nodeHandle_ , limits_))
      {
        ROS_FATAL("[MOTORS]: Joint Limits not specified in the parameter server");
        exit(-1);
      }
      // Register handle in joint limits interface
      joint_limits_interface::VelocityJointSoftLimitsHandle handle(
        jointLimitsHandle,  // We read the state and read/write the command
        limits_,  // Limits spec
        softLimits_);  // Soft limits spec.Not required in our implementation.

      velocityLimitsInterface_.registerHandle(handle);
    }
    registerInterface(&velocityLimitsInterface_);  // to do or not to do ???


    motorCurrentsMsg_.name.push_back(
      "Node 1, Left_Front Motor, EPOS2 Gateway");
    motorCurrentsMsg_.current.push_back(0);
    motorCurrentsMsg_.name.push_back(
      "Node 2, Left_Rear Motor, EPOS2 controller");
    motorCurrentsMsg_.current.push_back(0);
    motorCurrentsMsg_.name.push_back(
      "Node 3, Right_Front Motor, EPOS2 controller");
    motorCurrentsMsg_.current.push_back(0);
    motorCurrentsMsg_.name.push_back(
      "Node 4, Right_Rear Motor, EPOS2 controller.");
    motorCurrentsMsg_.current.push_back(0);
    currentPub_ = nodeHandle_.advertise<MotorCurrentsMsg>("/motors/current", 1);
  }

  MotorHardwareInterface::~MotorHardwareInterface()
  {
    delete motors_;
  }

  void MotorHardwareInterface::read(const ros::Duration& period)
  {
    int velFeed[jointNames_.size()];
    int currFeed[jointNames_.size()];
    double effortFeed[jointNames_.size()];

    for (int ii = 0; ii < jointNames_.size(); ii)
    {
      motors_->getRPM(jointToControllerMap_[jointNames_[ii]], &velFeed[ii]);
      motors_->getCurrent(jointToControllerMap_[jointNames_[ii]], &currFeed[ii]);
      motors_->getTorque(jointToControllerMap_[jointNames_[ii]], &effortFeed[ii]);
    }

    // invert right side motors feedback
    velFeed[1] *= -1;
    velFeed[3] *= -1;
    currFeed[1] *= -1;
    currFeed[3] *= -1;
    effortFeed[1] *= -1;
    effortFeed[3] *= -1;

    // Update local velocity, current, and position values
    for (int ii = 0; ii < 4; ii++)
    {
      velocity_[ii] =
        static_cast<double>(velFeed[ii]) / gearboxRatio_ / 30 * 3.14;
      current_[ii] = static_cast<double>(currFeed[ii]);
      motorCurrentsMsg_.current[ii] = current_[ii];
      position_[ii] = position_[ii] + period.toSec() * velocity_[ii];
      effort_[ii] = effortFeed[ii];
    }

    // Publish motors currents at the corresponding topic
    motorCurrentsMsg_.header.stamp = ros::Time::now();
    currentPub_.publish(motorCurrentsMsg_);
  }

  void MotorHardwareInterface::write()
  {
    if (motors_->getMode() == 0)
    {
      // why period needed ?
      ros::Duration period(0.1);
      velocityLimitsInterface_.enforceLimits(period);

      // Velocity Control Mode
      double RPMCommand[jointNames_.size()];

      for (int ii = 0; ii < jointNames_.size(); ii++)
      {
        RPMCommand[ii] = velCommand_[ii] * gearboxRatio_ * 30 / 3.14;
        if (fabs(RPMCommand[ii]) > maxRPM_)
        {
          ROS_DEBUG_STREAM("Limiting wheel speed, it's too high");
          RPMCommand[ii] = copysign(maxRPM_, RPMCommand[ii]);
        }
      }
      ROS_DEBUG_STREAM("Commands: " <<
        RPMCommand[0] << ", " <<
        RPMCommand[1] << "," <<
        RPMCommand[2] << "," <<
        RPMCommand[3])
        ;

      for (int ii = 0; ii < jointNames_.size(); ii++)
        motors_->writeRPM(jointNames_[ii], RPMCommand[ii]);
    }

    else if (motors_->getMode() == 1)
    {
      for (int ii = 0; ii < jointNames_.size(); ii++)
      {
        // if torque cmd exceeds limits, limit it to max torque value
        if (fabs(torqueCommand_[ii] > maxTorque_))
          torqueCommand_[ii] = copysign(maxTorque_, torqueCommand_[ii]);

        motors_->writeTorque(jointNames_[ii], torqueCommand_[ii]);
      }
      ROS_DEBUG_STREAM("Torque Commands: " <<
        torqueCommand_[0] << ", " <<
        torqueCommand_[1] << ", " <<
        torqueCommand_[2] << ", " <<
        torqueCommand_[3]);
    }
  }
}  // namespace motor
}  // namespace pandora_hardware_interface
