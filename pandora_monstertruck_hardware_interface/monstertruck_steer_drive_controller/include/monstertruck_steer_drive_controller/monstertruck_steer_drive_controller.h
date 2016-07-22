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
#include <pluginlib/class_list_macros.h>

namespace monstertruck_steer_drive_controller
{

#define CRAB_STEER_MODE 1
#define COUNTER_STEER_MODE -1

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

  /*
   * brief Computes a steer drive command from speed and turning radius
   * details counter steer => opposite front and rear steer angles
   * param speed [double] : the linear speed of the vehicle
   * param turningRadius [double] : the turning radius of the robot trajectory
   * return void
   */
  void computeCounterSteerCommand(double speed, double turningRadius);

  /*
   * brief Computes a parallel steer command using vx and vy
   * details parallel steer => equal front and rear steer angles
   * param vx [double] : the longitudinal velocity
   * param vy [double] : the lateral velocity
   * return void
   */
  void computeParallelSteerCommand(double vx, double vy);

  /**
   * brief Loads joint names and parameters from the parameter server
   * return void
   */
  void loadJointNamesAndParameters(ros::NodeHandle& nodeHandle);

 private:
  //!< left wheel joint vector
  std::vector<std::string> leftWheelJointNames_;
  //!< right wheel joint vector
  std::vector<std::string> rightWheelJointNames_;
  //!< left wheel joint handle vector
  std::vector<hardware_interface::JointHandle> leftWheelJoints_;
  //!< right wheel joint handle vector
  std::vector<hardware_interface::JointHandle> rightWheelJoints_;

  //!< left steer joint names
  std::vector<std::string> leftSteerJointNames_;
  //!< right steer joint names
  std::vector<std::string> rightSteerJointNames_;
  //!< left steer joints
  std::vector<hardware_interface::JointHandle> leftSteerJoints_;
  //!< right steer joints
  std::vector<hardware_interface::JointHandle> rightSteerJoints_;

  /**
   * brief steer drive command struct
   */
  struct SteerDriveCommand
  {
    //!< velocity of left wheels
    double leftVelocity;
    //!< velocity of right wheels
    double rightVelocity;
    //!< steer angle of left wheels
    double leftSteerAngle;
    //!< steer angle of right wheels
    double rightSteerAngle;
    //!< time stamp
    ros::Time stamp;
    //!< steer mode: CRAB_STEER_MODE or COUNTER_STEER_MODE
    int mode;

    SteerDriveCommand() :
      leftVelocity(0),
      rightVelocity(0),
      leftSteerAngle(0),
      rightSteerAngle(0),
      stamp(0),
      mode(1)
    {}
  };
  SteerDriveCommand steerDriveCommand_;

  //!< command subscribers
  std::vector<ros::Subscriber> commandSubscriber_;
  //!< command topics
  std::vector<std::string> commandTopic_;

  //!< command timeout
  double commandTimeout_;

  //!< wheels radius of the vehicle
  double wheelRadius_;
  //!< front to rear wheel centers distance
  double wheelbase_;
  // left to right wheel centers distance
  double track_;
  //!< minimum turning radius of the robot
  double minTurningRadius_;
  //!< maximum steer angle
  double maxSteerAngle_;
};

PLUGINLIB_EXPORT_CLASS(
  monstertruck_steer_drive_controller::MonstertruckSteerDriveController,
  controller_interface::ControllerBase);

}  // namespace monstertruck_steer_drive_controller

#endif  // MONSTERTRUCK_STEER_DRIVE_CONTROLLER_MONSTERTRUCK_STEER_DRIVE_CONTROLLER_H
