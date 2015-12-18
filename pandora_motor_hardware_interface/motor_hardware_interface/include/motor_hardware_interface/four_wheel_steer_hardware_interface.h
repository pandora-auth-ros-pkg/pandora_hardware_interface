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
**********************************************************************/
#ifndef MOTOR_HARDWARE_INTERFACE_FOUR_WHEEL_STEER_HARDWARE_INTERFACE_H
#define MOTOR_HARDWARE_INTERFACE_FOUR_WHEEL_STEER_HARDWARE_INTERFACE_H

#include "epos2_handler/serial_epos2_handler.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>
#include <ros/ros.h>

namespace pandora_hardware_interface
{
namespace motor
{
  class FourWheelSteerHardwareInterface : public hardware_interface::RobotHW
  {
    public:
      explicit FourWheelSteerHardwareInterface(ros::NodeHandle nodeHandle);
      ~FourWheelSteerHardwareInterface();
      void read(const ros::Duration& period);
      void write();

    private:
      void steerActuatorJointStateCallback(
        const dynamixel_msgs::JointStateConstPtr& msg, int id);
      bool loadJointConfiguration();

    private:
      ros::NodeHandle nodeHandle_;
      boost::scoped_ptr<SerialEpos2Handler> motorHandler_;

      // steer actuator command publishers
      std::vector<std::string> steerActuatorCommandTopics_;
      std::vector<ros::Publisher> steerActuatorCommandPublishers_;
      // steer actuator feedback subscribers
      std::vector<std::string> steerActuatorJointStateTopics_;
      std::vector<ros::Subscriber> steerActuatorJointStateSubscribers_;

      // joint state, velocity and position joint interfaces
      hardware_interface::JointStateInterface jointStateInterface_;
      hardware_interface::VelocityJointInterface velocityJointInterface_;
      hardware_interface::PositionJointInterface positionJointInterface_;

      // wheel drive joint names, variables and limits
      std::vector<std::string> wheelDriveJointNames_;
      double* wheelDriveVelocityCommand_;
      double* wheelDrivePosition_;
      double* wheelDriveVelocity_;
      double* wheelDriveEffort_;
      double wheelDriveMinVelocity_;
      double wheelDriveMaxVelocity_;

      // wheel steer joint names, variables and limits
      std::vector<std::string> wheelSteerJointNames_;
      double* wheelSteerPositionCommand_;
      double* wheelSteerPositionFeedback_;
      double* wheelSteerPosition_;
      double* wheelSteerVelocity_;
      double* wheelSteerEffort_;
      double wheelSteerMinPosition_;
      double wheelSteerMaxPosition_;

      // drive motor joints and parameters
      std::vector<std::string> driveMotorControllerNames_;
      std::vector<double> driveMotorRatio_;
      std::vector<double> driveMotorMinRPM_;
      std::vector<double> driveMotorMaxRPM_;

      // steer motor joints' names and variables
      std::vector<std::string> steerActuatorJointNames_;
      std::vector<double> steerActuatorMinPosition_;
      std::vector<double> steerActuatorMaxPosition_;

      // Steering Mechanism Polynomial Approximation Coefficients:

      // left to right angle polynomial coefficients
      std::vector<double> pLRCoeffs_;
      // right to left angle polynomial coefficients
      std::vector<double> pRLCoeffs_;
      // left front angle to front actuator angle
      std::vector<double> pLFFACoeffs_;
      // right rear angle to rear actuator angle
      std::vector<double> pRRRACoeffs_;
      // front actuator angle to left front angle
      std::vector<double> pFALFCoeffs_;
      // rear actuator angle to right rear angle
      std::vector<double> pRARRCoeffs_;
  };
}  // namespace motor
}  // namespace pandora_hardware_interface
#endif  // MOTOR_HARDWARE_INTERFACE_FOUR_WHEEL_STEER_HARDWARE_INTERFACE_H
