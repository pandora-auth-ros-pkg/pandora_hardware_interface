
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
     * param speed [double] : the linear speed of the vehicle
     * param turningRadius [double] : the turning radius of the robot trajectory
     * return void
     */
    void computeSteerDriveCommand(double speed, double turningRadius);

    /**
     * brief Loads joint names and parameters from the parameter server
     * return void
     */
    void loadJointNamesAndParameters(ros::NodeHandle& nodeHandle);

   private:
    // wheel joints
    std::vector<std::string> leftWheelJointNames_;
    std::vector<std::string> rightWheelJointNames_;
    std::vector<hardware_interface::JointHandle> leftWheelJoints_;
    std::vector<hardware_interface::JointHandle> rightWheelJoints_;

    // steer joints
    std::vector<std::string> leftSteerJointNames_;
    std::vector<std::string> rightSteerJointNames_;
    std::vector<hardware_interface::JointHandle> leftSteerJoints_;
    std::vector<hardware_interface::JointHandle> rightSteerJoints_;

    // steer drive command struct
    struct SteerDriveCommand
    {
      double leftVelocity;
      double rightVelocity;
      double leftSteerAngle;
      double rightSteerAngle;
      ros::Time stamp;

      SteerDriveCommand() :
        leftVelocity(0),
        rightVelocity(0),
        leftSteerAngle(0),
        rightSteerAngle(0),
        stamp(0)
      {}
    };
    SteerDriveCommand steerDriveCommand_;

    // command subscribers (only one active at a time)
    ros::Subscriber commandSubscriber_;
    std::string commandMessageType_;
    std::string commandTopic_;

    // command timeout
    double commandTimeout_;

    // vehicle base parameters
    double wheelRadius_;  // wheel radius of the vehicle
    double wheelbase_;  // front to rear wheel centers distance
    double track_;  // left to right wheel centers distance
};

PLUGINLIB_EXPORT_CLASS(
  monstertruck_steer_drive_controller::MonstertruckSteerDriveController,
  controller_interface::ControllerBase);

}  // namespace monstertruck_steer_drive_controller
#endif  // MONSTERTRUCK_STEER_DRIVE_CONTROLLER_MONSTERTRUCK_STEER_DRIVE_CONTROLLER_H
