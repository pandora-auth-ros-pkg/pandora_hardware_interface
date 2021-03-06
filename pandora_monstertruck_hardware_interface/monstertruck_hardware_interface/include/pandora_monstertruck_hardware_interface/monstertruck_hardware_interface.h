/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, P.A.N.D.O.R.A. Team.
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
#ifndef PANDORA_MONSTERTRUCK_HARDWARE_INTERFACE_MONSTERTRUCK_HARDWARE_INTERFACE_H
#define PANDORA_MONSTERTRUCK_HARDWARE_INTERFACE_MONSTERTRUCK_HARDWARE_INTERFACE_H

#include "epos2_handler/serial_epos2_handler.h"
#include "pololu_maestro/pololu_maestro.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

namespace pandora_hardware_interface
{
namespace monstertruck
{

class MonstertruckHardwareInterface : public hardware_interface::RobotHW
{
  public:
    explicit MonstertruckHardwareInterface(ros::NodeHandle nodeHandle);
    ~MonstertruckHardwareInterface();
    void read(const ros::Duration& period);
    void write();

  private:
    bool loadJointConfiguration();

    ros::NodeHandle nodeHandle_;
    boost::scoped_ptr<motor::SerialEpos2Handler> motorHandler_;
    boost::scoped_ptr<pololu_maestro::PololuMaestro> servoHandler_;

    ros::Publisher batteryVoltagePub_;

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
    double* wheelSteerPosition_;
    double* wheelSteerVelocity_;
    double* wheelSteerEffort_;
    double wheelSteerMinPosition_;
    double wheelSteerMaxPosition_;

    // drive motor joints and parameters
    std::string motorControllerName_;
    int32_t  motorRatio_;
    int32_t  motorMinRPM_;
    int32_t motorMaxRPM_;

    // vehicle parameters
    double wheelbase_;  // the wheel longitudinal separation
    double track_;  // the wheel lateral separation

    // Steering Mechanism Polynomial Approximation Coefficients:

    // left front to right front angle polynomial coefficients
    std::vector<double> pLFRFCoeffs_;
    // left rear to left rear angle polynomial coefficients
    std::vector<double> pLRRRCoeffs_;
    // left front angle to front actuator angle
    std::vector<double> pLFFACoeffs_;
    // left rear angle to rear actuator angle
    std::vector<double> pLRRACoeffs_;
    // front actuator angle to left front angle
    std::vector<double> pFALFCoeffs_;
    // rear actuator angle to left rear angle
    std::vector<double> pRALRCoeffs_;
};  // class MonstertruckHardwareInterface

}  // namespace monstertruck
}  // namespace pandora_hardware_interface
#endif  // PANDORA_MONSTERTRUCK_HARDWARE_INTERFACE_MONSTERTRUCK_HARDWARE_INTERFACE_H
