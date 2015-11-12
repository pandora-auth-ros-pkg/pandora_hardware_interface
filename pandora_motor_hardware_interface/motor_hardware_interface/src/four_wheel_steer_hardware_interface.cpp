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

#include "motor_hardware_interface/motor_hardware_interface.h"

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
    nodeHandle_.getParam("max_RPM", maxRPM);
    nodeHandle_.getParam("gearbox_ratio", gearboxRatio_);
    nodeHandle_.getParam("wheel_ratio", wheelRatio_);
    nodeHandle_.getParam("motor_joint", motorJointName_);

    // connect and register the motor joint state interface
    position_ = 0;
    velocity_ = 0;
    effort_ = 0;
    hardware_interface::JointStateHandle jointStateHandle(
      motorJointName_,
      &position_,
      &velocity_,
      &effort_);
    jointStateInterface_.registerHandle(jointStateHandle);
    registerInterface(&jointStateInterface_);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle jointVelocityHandle(
      jointStateInterface_.getHandle(motorJointName_),
      &velocityCommand_);
    registerInterface(&velocityJointInterface_);

    // initialize joint limits
    // TODO(gkouros): reevaluate use of limit interface
    hardware_interface::JointHandle jointLimitsHandle =
      velocityJointInterface_getHandle_(jointNames_[ii]);
    if (!joint_limits_interface::getJointLimits(
        motorJointName_, nodeHandle_, limits_))
    {
      ROS_FATAL("[MOTOR]: Joint Limits not specified in parameter server");
      exit(-1);
    }
    // Register handle in joint limits interface
    joint_limits_interface::VelocityJointSoftLimitsHandle handle(
      jointLimitsHandle,
      limits_,
      softLimits_);

    velocityLimitsInterface_.registerHandle(handle);
    registerInterface(&velocityLimitsInterface);
  }


  FourWheelSteerHardwareInterface::~FourWheelSteerHardwareInterface()
  {
    delete motorHandler_;
  }


  FourWheelSteerHardwareInterface::read(const ros::Duration& period)
  {
   //TODO(gkouros): read motor velocity feedback
   //TODO(gkouros): read motor torque feedback
   //TODO(gkouros): publish current msg
  }

  FourWheelSteerHardwareInterface::write()
  {
    //TODO(gkouros): enforce limits
    //TODO(gkouros): enforce transmission expression
    //TODO(gkouros): write rpm to motor 
    //TODO(gkouros): publish front_steer_angle, rear_steer_angle
  }

}  // namespace motor
}  // namespace pandora_hardware_interface
