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

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "epos_handler/serial_epos_handler.h"
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include "ros/ros.h"

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
      SerialEposHandler motorHandler_;
      ros::NodeHandle nodeHandle_;

      // interfaces
      hardware_interface::JointStateInterface jointStateInterface_;
      hardware_interface::VelocityJointInterface velocityJointInterface_;

      // joint limits interfaces
      joint_limits_interface::VelocityJointSoftLimitsInterface
        velocityLimitsInterface_;

      // joint limits
      joint_limits_interface::JointLimits limits_;
      joint_limits_interface::SoftJointLimits softLimits_;

      // Interface Variables
      std::string motorJointName_;
      double velocityCommand_;
      double position_;
      double velocity_;
      double effort_;
      double maxRPM_;
      double gearboxRatio_;
      double wheelRatio_;
  };
}  // namespace motor
}  // namespace pandora_hardware_interface
#endif  // MOTOR_HARDWARE_INTERFACE_FOUR_WHEEL_STEER_HARDWARE_INTERFACE_H
