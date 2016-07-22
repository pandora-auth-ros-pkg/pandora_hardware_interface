/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, P.A.N.D.O.R.A Team
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
 *   * Neither the name of the PAL Robotics nor the names of its
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
 *  Author: George Kouros
 *********************************************************************/

#ifndef MONSTERTRUCK_STEER_DRIVE_CONTROLLER_MONSTERTRUCK_STEER_DRIVE_CONTROLLER_H
#define MONSTERTRUCK_STEER_DRIVE_CONTROLLER_MONSTERTRUCK_STEER_DRIVE_CONTROLLER_H

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <dual_controller_interface/dual_controller_interface.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.h>

#include "monstertruck_steer_drive_controller/odometry.h"

namespace monstertruck_steer_drive_controller
{

/**
 * class MonstertruckSteerDriveController
 * brief Controller for a monstertruck bi-steerable mobile robot
 */
class MonstertruckSteerDriveController : public
  dual_controller_interface::DualController<
    hardware_interface::VelocityJointInterface,
    hardware_interface::PositionJointInterface>
{
 public:
  /**
   * brief Constructor
   */
  MonstertruckSteerDriveController();

  /**
   * brief Initialize controller
   * param hw [hardware_interface::VelocityJointInterface*]: wheel velocity
   *  joint interface
   * param nodeHandle [ros::NodeHandle &]: node handle inside the controller
   *  namespace
   */
  bool init(
    hardware_interface::VelocityJointInterface* velJointInterface,
    hardware_interface::PositionJointInterface* posJointInterface,
    ros::NodeHandle& nodeHandle);

  /**
   * brief Updates controller
   * param time [ros::Time& time]: current time
   * param period [ros::Time& time]: time elapsed since last update
   * return void
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * brief Starts controller
   * param time [ros::Time&]: current time
   * return void
   */
  void starting(const ros::Time& time);

  /**
   * brief Stops controller
   * param time [ros::Time&]: current time
   * return void
   */
  void stopping(const ros::Time& time);

 private:
  /**
   * brief Set all wheel speeds at zero
   * return void
   */
  void brake();

  /**
   * brief Aligns all wheels
   * return void
   */
  void alignWheels();

  /**
   * brief Callback for robot twist command msgs
   * command [const geometry_msgs::Twist&]: robot twist command
   * return void
   */
  void twistCommandCallback(
    const geometry_msgs::TwistConstPtr& msg);

  /**
   * brief Callback for robot ackermann drive command msgs
   * command [const ackermann_msgs::AckermannDrive&]: robot twist command
   * return void
   */
  void ackermannDriveCommandCallback(
    const ackermann_msgs::AckermannDriveConstPtr& msg);

  /**
   * @brief Convert base ackeramnn command to wheel velocities and steering angles
   * @param speed [double] : base speed
   * @param steeringAngle [double] : front steering angle
   */
  void convertBase2JointCmds(double speed, double steeringAngle);

  /**
   * @brief Convert base twist command to wheel velocities and steering angles
   * @param linVelX [double] : base longitudinal linear velocity
   * @param linVelY [double] : base lateral linear velocity
   * @param angVel [double] : base angular velocity
   */
  void convertBase2JointCmds(double linVelX, double linVelY, double angVel);

  /**
   * brief Loads joint names and parameters from the parameter server
   * return void
   */
  void loadParams();

 private:

  /**
   * brief steer drive command struct
   */
  struct SteerDriveCommand
  {
    double leftFrontWheelVelocity;
    double leftRearWheelVelocity;
    double rightFrontWheelVelocity;
    double rightRearWheelVelocity;
    double frontSteeringAngle;
    double rearSteeringAngle;
    ros::Time stamp;

    SteerDriveCommand() :
      leftFrontWheelVelocity(0.0),
      leftRearWheelVelocity(0.0),
      rightFrontWheelVelocity(0.0),
      rightRearWheelVelocity(0.0),
      frontSteeringAngle(0),
      rearSteeringAngle(0),
      stamp(0)
    {}
  };

  SteerDriveCommand steerDriveCommand_;

  //!< ros node handle
  ros::NodeHandle *pnh_;

  // drive joint names
  std::string leftFrontDriveJointName_;
  std::string leftRearDriveJointName_;
  std::string rightFrontDriveJointName_;
  std::string rightRearDriveJointName_;
  // steer joint names
  std::string leftFrontSteerJointName_;
  std::string leftRearSteerJointName_;
  std::string rightFrontSteerJointName_;
  std::string rightRearSteerJointName_;

  // drive joint handles
  hardware_interface::JointHandle leftFrontDriveJoint_;
  hardware_interface::JointHandle leftRearDriveJoint_;
  hardware_interface::JointHandle rightFrontDriveJoint_;
  hardware_interface::JointHandle rightRearDriveJoint_;
  // steer joint handles
  hardware_interface::JointHandle leftFrontSteerJoint_;
  hardware_interface::JointHandle leftRearSteerJoint_;
  hardware_interface::JointHandle rightFrontSteerJoint_;
  hardware_interface::JointHandle rightRearSteerJoint_;


  //!< twist command subscriber
  ros::Subscriber twistCmdSub_;
  //!< ackermann command subscriber
  ros::Subscriber ackCmdSub_;
  //!< command timeout
  double cmdTimeout_;

  // odometry related

  //!< publish period of odometry
  ros::Duration odomPubPeriod_;
  //!< last time odometry was published
  ros::Time lastOdomPubTime_;
  ros::Publisher odomPub_;
  nav_msgs::Odometry odom_;
  Odometry odometry_;

  // robot geometric parameterers

  //!< wheels radius of the vehicle
  double wheelRadius_;
  //!< front to rear wheel centers distance
  double wheelbase_;
  // left to right wheel centers distance
  double track_;
  //!< maximum steer angle
  double maxSteeringAngle_;
};

PLUGINLIB_EXPORT_CLASS(
  monstertruck_steer_drive_controller::MonstertruckSteerDriveController,
  controller_interface::ControllerBase);

}  // namespace monstertruck_steer_drive_controller

#endif  // MONSTERTRUCK_STEER_DRIVE_CONTROLLER_MONSTERTRUCK_STEER_DRIVE_CONTROLLER_H
