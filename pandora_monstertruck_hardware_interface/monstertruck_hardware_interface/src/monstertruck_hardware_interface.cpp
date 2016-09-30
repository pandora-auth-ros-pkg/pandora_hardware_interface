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
#include <std_msgs/Float64.h>

namespace pandora_hardware_interface
{
namespace monstertruck
{
  MonstertruckHardwareInterface::MonstertruckHardwareInterface(
    ros::NodeHandle nodeHandle)
  : nodeHandle_(nodeHandle)
  , wheelbase_(0.32)
  , track_(0.26)
  {
    // initialize motor handler
    motorHandler_.reset(
      new motor::SerialEpos2Handler());

    // initialize servo hanler
    servoHandler_.reset(
      new pololu_maestro::PololuMaestro("/dev/maestro_serial", 115200, 500));

    batteryVoltagePub_ =
      nodeHandle_.advertise<std_msgs::Float64>("/battery/voltage", 1);

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
    int32_t rpm;

    double frontAngle;
    double rearAngle;

    double frontLeftAngle = 0;
    double frontRightAngle = 0;
    double rearLeftAngle = 0;
    double rearRightAngle = 0;

    // read motor rpm
    motorHandler_->getRPM(motorControllerName_, &rpm);
    // reverse motor rotation direction
    rpm *= -1;

    // read servo handler errors to clear them
    servoHandler_->readErrors();

    // read wheel steer actuator positions
    frontAngle = servoHandler_->readPosition(1) - M_PI/2;
    rearAngle = servoHandler_->readPosition(0) - M_PI/2;

    // compute wheel steer angles from actuator steer angles
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

    // compute wheel velocities
    double frontSteeringAngle = atan(
      2 / (1/tan(frontLeftAngle) + 1/tan(frontRightAngle)));
    double rearSteeringAngle = atan(
      2 / (1/tan(rearLeftAngle) + 1/tan(rearRightAngle)));
    double beta = atan((tan(frontSteeringAngle) + tan(rearSteeringAngle))
      / wheelbase_);
    double turningRadius = wheelbase_ / fabs(cos(beta)
      * (tan(frontSteeringAngle) - tan(rearSteeringAngle)));
    double velocity =  static_cast<double>(rpm) / motorRatio_ / 60 * 2 * M_PI;
    wheelDriveVelocity_[0] = velocity * (turningRadius - track_/2)
      / turningRadius / cos(frontLeftAngle);
    wheelDriveVelocity_[1] = velocity * (turningRadius + track_/2)
      / turningRadius / cos(rearLeftAngle);
    wheelDriveVelocity_[2] = velocity * (turningRadius - track_/2)
      / turningRadius / cos(frontRightAngle);
    wheelDriveVelocity_[3] = velocity * (turningRadius + track_/2)
      / turningRadius / cos(rearRightAngle);

    // update wheel steer angles
    wheelSteerPosition_[0] = frontLeftAngle;
    wheelSteerPosition_[1] = rearLeftAngle;
    wheelSteerPosition_[2] = frontRightAngle;
    wheelSteerPosition_[3] = rearRightAngle;

    // read battery voltage from pololu maestro and publish it
    std_msgs::Float64 batteryVoltage;
    batteryVoltage.data = servoHandler_->readVoltage(5) * 10;
    batteryVoltagePub_.publish(batteryVoltage);
  }


  void MonstertruckHardwareInterface::write()
  {
    // compute motor rpm
    double velocityCmd = 0;
    for (int ii = 0; ii < wheelDriveJointNames_.size(); ii++)
      velocityCmd +=
        wheelDriveVelocityCommand_[ii] / wheelDriveJointNames_.size();

    int32_t rpmCmd =
      static_cast<int32_t>(velocityCmd * motorRatio_ * 60 / 2 / M_PI);

    // compute front and rear steering angles
    double frontSteeringAngleCmd =
      (wheelSteerPositionCommand_[0] + wheelSteerPositionCommand_[2]) / 2;
    double rearSteeringAngleCmd =
      (wheelSteerPositionCommand_[1] + wheelSteerPositionCommand_[3]) / 2;

    // enforce limits
    rpmCmd = std::min(std::max(rpmCmd, motorMinRPM_), motorMaxRPM_);
    frontSteeringAngleCmd =
      std::min(std::max(frontSteeringAngleCmd, -M_PI/4), M_PI/4);
    rearSteeringAngleCmd =
      std::min(std::max(rearSteeringAngleCmd, -M_PI/4) , M_PI/4);

    // read servo handler errors to clear them
    servoHandler_->readErrors();

    // write commands
    motorHandler_->writeRPM(motorControllerName_, -rpmCmd);
    servoHandler_->setTarget(1, frontSteeringAngleCmd + M_PI/2);
    servoHandler_->setTarget(0, rearSteeringAngleCmd + M_PI/2);
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

    if (nodeHandle_.getParam("motor/controller_name",
        motorControllerName_)
      && nodeHandle_.getParam("motor/ratio", motorRatio_)
      && nodeHandle_.getParam("motor/min_rpm", motorMinRPM_)
      && nodeHandle_.getParam("motor/max_rpm", motorMaxRPM_))
    {
      ROS_INFO("[MOTORS] motor parameters loaded successfully!");
    }
    else
    {
      ROS_INFO(
        "[MOTORS] motor parameters could not be loaded! Exiting...");
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
