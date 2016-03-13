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
*********************************************************************/

#include "pandora_monstertruck_hardware_interface/monstertruck_hardware_interface.h"
#include <math.h>
#include <algorithm>

namespace pandora_hardware_interface
{
namespace monstertruck
{
  MonstertruckHardwareInterface::MonstertruckHardwareInterface(
    ros::NodeHandle nodeHandle)
   :
    nodeHandle_(nodeHandle)
  {
    motorHandler_.reset(
      new motor::SerialEpos2Handler());
    servoHandler_.reset(
      new pololu_maestro::PololuMaestro("/dev/ttyACM0", 9600, 500));

    // load joint names from parameter server
    loadJointConfiguration();

    // allocate memory for variables of drive and steer joints
    wheelDriveVelocityCommand_ = new double[wheelDriveJointNames_.size()];
    wheelDrivePosition_ = new double[wheelDriveJointNames_.size()];
    wheelDriveVelocity_ = new double[wheelDriveJointNames_.size()];
    wheelDriveEffort_ = new double[wheelDriveJointNames_.size()];

    wheelSteerPositionCommand_ = new double[wheelSteerJointNames_.size()];
    wheelSteerPosition_ = new double[wheelSteerJointNames_.size()];
    wheelSteerVelocity_ = new double[wheelSteerJointNames_.size()];
    wheelSteerEffort_ = new double[wheelSteerJointNames_.size()];

    // create and register the joint state handles for the drive joints
    for (int ii = 0; ii < wheelDriveJointNames_.size(); ii++)
    {
      wheelDrivePosition_[ii] = 0;
      wheelDriveVelocity_[ii] = 0;
      wheelDriveEffort_[ii] = 0;
      hardware_interface::JointStateHandle jointStateHandle(
        wheelDriveJointNames_[ii],
        &wheelDrivePosition_[ii],
        &wheelDriveVelocity_[ii],
        &wheelDriveEffort_[ii]);
      jointStateInterface_.registerHandle(jointStateHandle);
    }

    // create and register the joint state handles for the steer joints
    for (int ii = 0; ii < wheelSteerJointNames_.size(); ii++)
    {
      wheelSteerPosition_[ii] = 0;
      wheelSteerVelocity_[ii] = 0;
      wheelSteerEffort_[ii] = 0;
      hardware_interface::JointStateHandle jointStateHandle(
        wheelSteerJointNames_[ii],
        &wheelSteerPosition_[ii],
        &wheelSteerVelocity_[ii],
        &wheelSteerEffort_[ii]);
      jointStateInterface_.registerHandle(jointStateHandle);
    }

    // register the joint state interface
    registerInterface(&jointStateInterface_);

    // connect and register the joint velocity interface for the drive joints
    for (int ii = 0; ii < wheelDriveJointNames_.size(); ii++)
    {
      wheelDriveVelocityCommand_[ii] = 0;
      hardware_interface::JointHandle jointHandle(
        jointStateInterface_.getHandle(wheelDriveJointNames_[ii]),
        &wheelDriveVelocityCommand_[ii]);
      velocityJointInterface_.registerHandle(jointHandle);
    }
    registerInterface(&velocityJointInterface_);

    // connect and register the joint position interface for the steer joints
    for (int ii = 0; ii < wheelSteerJointNames_.size(); ii++)
    {
      wheelSteerPositionCommand_[ii] = 0;
      hardware_interface::JointHandle jointHandle(
        jointStateInterface_.getHandle(wheelSteerJointNames_[ii]),
        &wheelSteerPositionCommand_[ii]);
      positionJointInterface_.registerHandle(jointHandle);
    }
    registerInterface(&positionJointInterface_);

    ROS_INFO("[MONSTERTRUCK] Node Initialized");
  }

  MonstertruckHardwareInterface::~MonstertruckHardwareInterface()
  {
    delete wheelDriveVelocityCommand_;
    delete wheelDrivePosition_;
    delete wheelDriveVelocity_;
    delete wheelDriveEffort_;
    delete wheelSteerPositionCommand_;
    delete wheelSteerPosition_;
    delete wheelSteerVelocity_;
    delete wheelSteerEffort_;
  }


  void MonstertruckHardwareInterface::read(const ros::Duration& period)
  {
    // read wheel drive joint velocities
    double velocity[driveMotorControllerNames_.size()];

    for (int ii = 0; ii < driveMotorControllerNames_.size(); ii++)
    {
      int32_t rpm;
      motorHandler_->getRPM(driveMotorControllerNames_[ii], &rpm);
      velocity[ii] = rpm / driveMotorRatio_[ii] / 60 * 2 * M_PI;
    }

    if (driveMotorControllerNames_.size() == 1)
    {
      for (int ii = 0; ii < wheelDriveJointNames_.size(); ii++)
        wheelDriveVelocity_[ii] = velocity[0];
    }
    else if (driveMotorControllerNames_.size() == wheelDriveJointNames_.size())
    {
      for (int ii = 0; ii < wheelDriveJointNames_.size(); ii++)
        wheelDriveVelocity_[ii] = velocity[ii];
    }
    else
    {
      ROS_ERROR("[MOTORS] number of motors is %zu (should be 1 or 4)",
        driveMotorControllerNames_.size());
      exit(1);
    }

    // read wheel steer actuator positions
    double frontAngle = servoHandler_->readPosition(0);
    double rearAngle = servoHandler_->readPosition(1);

    double frontLeftAngle = 0;
    double frontRightAngle = 0;
    double rearLeftAngle = 0;
    double rearRightAngle = 0;

    // calculate wheel steer angles from actuator steer angles
    for (int ii = 0; ii < pFALFCoeffs_.size(); ii++)
      frontLeftAngle += pFALFCoeffs_[ii] *
        pow(frontAngle, pFALFCoeffs_.size() - ii - 1);

    for (int ii = 0; ii < pLFRFCoeffs_.size(); ii++)
      frontRightAngle += pLFRFCoeffs_[ii] *
        pow(frontLeftAngle, pLFRFCoeffs_.size() - ii - 1);

    for (int ii = 0; ii < pRARRCoeffs_.size(); ii++)
      rearRightAngle += pRARRCoeffs_[ii] *
        pow(rearAngle, pRARRCoeffs_.size() - ii - 1);

    for (int ii = 0; ii < pRRLRCoeffs_.size(); ii++)
      rearLeftAngle += pRRLRCoeffs_[ii] *
        pow(rearRightAngle, pRRLRCoeffs_.size() - ii - 1);

    // update wheel steer angles
    wheelSteerPosition_[0] = frontLeftAngle;
    wheelSteerPosition_[1] = rearLeftAngle;
    wheelSteerPosition_[2] = frontRightAngle;
    wheelSteerPosition_[3] = rearRightAngle;
  }


  void MonstertruckHardwareInterface::write()
  {
    double velocity[driveMotorControllerNames_.size()];

    if (driveMotorControllerNames_.size() == 1)
    {
      for (int ii = 0; ii < wheelDriveJointNames_.size(); ii++)
        velocity[0] +=
          fabs(wheelDriveVelocityCommand_[ii] / wheelDriveJointNames_.size());
        velocity[0] = copysign(velocity[0], wheelDriveVelocityCommand_[0]);
    }
    else if (driveMotorControllerNames_.size() == wheelDriveJointNames_.size())
    {
      for (int ii = 0; ii < wheelDriveJointNames_.size(); ii++)
        velocity[ii] = wheelDriveVelocityCommand_[ii];
    }
    else
    {
      ROS_ERROR("[MOTORS] number of motors is %zu (should be 1 or 4)",
        driveMotorControllerNames_.size());
      exit(1);
    }

    for (int ii = 0; ii < driveMotorControllerNames_.size(); ii++)
      motorHandler_->writeRPM(driveMotorControllerNames_[ii],
        static_cast<int32_t>(driveMotorRatio_[ii] * velocity[ii] * 60 / 2 / M_PI));

    // compute the front actuator steer angle, using the left front wheel steer
    // angle and a polynomial approximation of the relationship between the two
    double steerActuatorCmd[2] = {0, 0};
    for (int ii = 0; ii < pLFFACoeffs_.size(); ii++)
    {
      steerActuatorCmd[0] += pLFFACoeffs_[ii] *
        pow(wheelSteerPositionCommand_[0], pLFFACoeffs_.size() - ii - 1);
    }

    // compute the rear actuator steer angle, using the right rear wheel steer
    // angle and a polynomial approximation of the relationship between the two
    for (int ii = 0; ii < pRRRACoeffs_.size(); ii++)
    {
      steerActuatorCmd[1] += pRRRACoeffs_[ii] *
        pow(wheelSteerPositionCommand_[3], pRRRACoeffs_.size() - ii - 1);
    }

    for (int ii = 0; ii < steerActuatorJointNames_.size(); ii++)
    {
      // enforce steer actuator command limits
      steerActuatorCmd[ii] =
        std::min(
          std::max(steerActuatorCmd[ii], steerActuatorMinPosition_[ii]),
          steerActuatorMaxPosition_[ii]);

      // write command to servoHandler
      servoHandler_->setTarget(static_cast<uint8_t>(ii), steerActuatorCmd[ii]);
    }
  }


  bool MonstertruckHardwareInterface::loadJointConfiguration()
  {
    if (nodeHandle_.getParam("wheel_drive_joints/names", wheelDriveJointNames_)
      && nodeHandle_.getParam("wheel_drive_joints/limits/min_velocity",
        wheelDriveMinVelocity_)
      && nodeHandle_.getParam("wheel_drive_joints/limits/max_velocity",
        wheelDriveMaxVelocity_)
      && nodeHandle_.getParam("wheel_steer_joints/names", wheelSteerJointNames_)
      && nodeHandle_.getParam("wheel_steer_joints/limits/min_position",
        wheelSteerMinPosition_)
      && nodeHandle_.getParam("wheel_steer_joints/limits/max_position",
        wheelSteerMaxPosition_))
    {
      ROS_INFO("[MOTORS] Drive and Steer joints loaded successfully!");
    }
    else
    {
      ROS_ERROR(
        "[MOTORS] drive and steer joints could not be loaded! Exiting...");
      exit(EXIT_FAILURE);
    }

    if (nodeHandle_.getParam("drive_motors/motor_controller_names",
        driveMotorControllerNames_)
      && nodeHandle_.getParam("drive_motors/ratios", driveMotorRatio_)
      && nodeHandle_.getParam("drive_motors/min_rpms", driveMotorMinRPM_)
      && nodeHandle_.getParam("drive_motors/max_rpms", driveMotorMaxRPM_))
    {
      ROS_INFO("[MOTORS] Drive motor parameters loaded successfully!");
    }
    else
    {
      ROS_INFO(
        "[MOTORS] Drive motors' parameters could not be loaded! Exiting...");
      exit(EXIT_FAILURE);
    }

    if (nodeHandle_.getParam("steer_actuators/joint_names",
        steerActuatorJointNames_)
      && nodeHandle_.getParam("steer_actuators/min_position",
        steerActuatorMinPosition_)
      && nodeHandle_.getParam("steer_actuators/max_position",
        steerActuatorMaxPosition_))
    {
      ROS_INFO("[MOTORS] Steer actuator parameters loaded successfully!");
    }
    else
    {
      ROS_ERROR(
        "[MOTORS] Steer actuator parameters could not be loaded! Exiting...");
      exit(EXIT_FAILURE);
    }

    if (nodeHandle_.getParam(
        "steer_mechanism/polynomial_approximation_coefficients/p_lf_to_rf",
        pLFRFCoeffs_)
      && nodeHandle_.getParam(
        "steer_mechanism/polynomial_approximation_coefficients/p_rr_to_lr",
        pRRLRCoeffs_)
      && nodeHandle_.getParam(
        "steer_mechanism/polynomial_approximation_coefficients/p_lf_to_fa",
        pLFFACoeffs_)
      && nodeHandle_.getParam(
        "steer_mechanism/polynomial_approximation_coefficients/p_rr_to_ra",
        pRRRACoeffs_)
      && nodeHandle_.getParam(
        "steer_mechanism/polynomial_approximation_coefficients/p_fa_to_lf",
        pFALFCoeffs_)
      && nodeHandle_.getParam(
        "steer_mechanism/polynomial_approximation_coefficients/p_ra_to_rr",
        pRARRCoeffs_))
    {
      ROS_INFO("[MOTORS] Steer mechanism parameters loaded successfully!");
    }
    else
    {
      ROS_ERROR(
        "[MOTORS] Steer mechanism parameters not loaded successfully");
    }
  }
}  // namespace monstertruck
}  // namespace pandora_hardware_interface
