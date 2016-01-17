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

#include <math.h>

namespace pandora_hardware_interface
{
namespace motor
{
  FourWheelSteerHardwareInterface::FourWheelSteerHardwareInterface(
    ros::NodeHandle nodeHandle)
   :
    nodeHandle_(nodeHandle)
  {
    motorHandler_.reset(new SerialEpos2Handler());

    // load joint names from parameter server
    loadJointConfiguration();

    // allocate memory for variables of drive and steer joints
    wheelDriveVelocityCommand_ = new double[wheelDriveJointNames_.size()];
    wheelDrivePosition_ = new double[wheelDriveJointNames_.size()];
    wheelDriveVelocity_ = new double[wheelDriveJointNames_.size()];
    wheelDriveEffort_ = new double[wheelDriveJointNames_.size()];

    wheelSteerPositionCommand_ = new double[wheelSteerJointNames_.size()];
    wheelSteerPositionFeedback_ = new double[wheelSteerJointNames_.size()];
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
      wheelSteerPositionFeedback_[ii] = 0;

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

    // generate a command publisher and a feedback subscriber for each actuator
    for (int ii = 0; ii < steerActuatorJointNames_.size(); ii++)
    {
      ros::Publisher pub =
        nodeHandle_.advertise<std_msgs::Float64>(
          steerActuatorCommandTopics_[ii], 1);

      // add publisher to corresponding vector
      steerActuatorCommandPublishers_.push_back(pub);

      ros::Subscriber sub = nodeHandle_.subscribe<std_msgs::Float64>(
        steerActuatorJointStateTopics_[ii],
        1,
        boost::bind(
          &FourWheelSteerHardwareInterface::steerActuatorFeedbackCallback,
          this,
          _1,
          ii));

      // add subscriber to corresponding vector
      steerActuatorJointStateSubscribers_.push_back(sub);
    }

    ROS_INFO("[MOTORS] Node Initialized");
  }

  FourWheelSteerHardwareInterface::~FourWheelSteerHardwareInterface()
  {
    delete wheelDriveVelocityCommand_;
    delete wheelDrivePosition_;
    delete wheelDriveVelocity_;
    delete wheelDriveEffort_;
    delete wheelSteerPositionCommand_;
    delete wheelSteerPositionFeedback_;
    delete wheelSteerPosition_;
    delete wheelSteerVelocity_;
    delete wheelSteerEffort_;
  }


  void FourWheelSteerHardwareInterface::read(const ros::Duration& period)
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

    // read wheel steer joint positions
    for (int ii = 0; ii < wheelSteerJointNames_.size(); ii++)
      wheelSteerPosition_[ii] = wheelSteerPositionFeedback_[ii];
  }


  void FourWheelSteerHardwareInterface::write()
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

    // publish the front and rear steer actuator commands to their
    // corresponding topics
    for (int ii = 0; ii < steerActuatorJointNames_.size(); ii++)
    {
      std_msgs::Float64 msg;
      msg.data = steerActuatorCmd[ii];

      // enforce steer actuator limits
      if (steerActuatorCmd[ii] > steerActuatorMaxPosition_[ii])
        steerActuatorCmd[ii] = steerActuatorMaxPosition_[ii];
      else if (steerActuatorCmd[ii] < steerActuatorMinPosition_[ii])
        steerActuatorCmd[ii] = steerActuatorMinPosition_[ii];

      // publish command
      steerActuatorCommandPublishers_[ii].publish(msg);
    }
  }


  void FourWheelSteerHardwareInterface::steerActuatorFeedbackCallback(
    const std_msgs::Float64ConstPtr& msg, int id)
  {
    double leftAngle = 0, rightAngle = 0;
    double actuatorSteerAngle = msg->data;  // steering actuator position

    if (id == 0)  // msg contains front actuator position
    {
      // compute left front wheel joint position, using the front actuator
      // angle and a polynomial approximation of the relationship between them
      for (int ii = 0; ii < pFALFCoeffs_.size(); ii++)
      {
        leftAngle += pFALFCoeffs_[ii] *
          pow(actuatorSteerAngle, pFALFCoeffs_.size() - ii - 1);
      }
      // compute right front wheel joint position, using the left front wheel
      // angle and a polynomial approximation of the relationship between them
      for (int ii = 0; ii < pLFRFCoeffs_.size(); ii++)
      {
        rightAngle += pLFRFCoeffs_[ii] *
          pow(leftAngle, pLFRFCoeffs_.size() - ii - 1);
      }

      // write results to corresponding contaner
      wheelSteerPositionFeedback_[0] = leftAngle;
      wheelSteerPositionFeedback_[2] = rightAngle;
    }
    else if (id == 1)  // msg contains rear actuator position
    {
      // compute right rear wheel joint position, using the rear actuator
      // angle and a polynomial approximation of the relationship between them
      for (int ii = 0; ii < pRARRCoeffs_.size(); ii++)
      {
        rightAngle += pRARRCoeffs_[ii] *
          pow(actuatorSteerAngle, pRARRCoeffs_.size() - ii - 1);
      }
      // compute left rear wheel joint position, using the right rear wheel
      // angle and a polynomial approximation of the relationship between them
      for (int ii = 0; ii < pRRLRCoeffs_.size(); ii++)
      {
        leftAngle += pRRLRCoeffs_[ii] *
          pow(rightAngle, pRRLRCoeffs_.size() - ii - 1);
      }

      // write results to corresponding container
      wheelSteerPositionFeedback_[1] = leftAngle;
      wheelSteerPositionFeedback_[3] = rightAngle;
    }
  }


  bool FourWheelSteerHardwareInterface::loadJointConfiguration()
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
      && nodeHandle_.getParam("steer_actuators/command_topics",
        steerActuatorCommandTopics_)
      && nodeHandle_.getParam("steer_actuators/joint_state_topics",
        steerActuatorJointStateTopics_)
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
}  // namespace motor
}  // namespace pandora_hardware_interface
