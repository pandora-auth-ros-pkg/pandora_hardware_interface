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

#include "monstertruck_steer_drive_controller/monstertruck_steer_drive_controller.h"
#include <cmath>

namespace monstertruck_steer_drive_controller
{

MonstertruckSteerDriveController::MonstertruckSteerDriveController()
  :
    steerDriveCommand_(),
    commandTimeout_(0.5),
    wheelRadius_(0),
    wheelbase_(0),
    track_(0)
{
  leftWheelJoints_.resize(2);
  rightWheelJoints_.resize(2);
  leftSteerJoints_.resize(2);
  rightSteerJoints_.resize(2);
}

bool MonstertruckSteerDriveController::init(
  hardware_interface::VelocityJointInterface* velJointInterface,
  hardware_interface::PositionJointInterface* posJointInterface,
  ros::NodeHandle& nodeHandle)
{
  // load joint names and parameters from the parameter server
  loadJointNamesAndParameters(nodeHandle);

  // compute min turning radius
  minTurningRadius_ = wheelbase_ / 2 / tan(maxSteerAngle_);

  // load joint handles
  for (int ii = 0; ii < 2; ii++)
  {
    leftWheelJoints_[ii] =
      velJointInterface->getHandle(leftWheelJointNames_[ii]);
    rightWheelJoints_[ii] =
      velJointInterface->getHandle(rightWheelJointNames_[ii]);

    leftSteerJoints_[ii] =
      posJointInterface->getHandle(leftSteerJointNames_[ii]);
    rightSteerJoints_[ii] =
      posJointInterface->getHandle(rightSteerJointNames_[ii]);
  }

  // initialize twist and ackermann cmd subscribers
  commandSubscriber_.push_back(
    nodeHandle.subscribe(
      commandTopic_[0], 1,
      &MonstertruckSteerDriveController::twistCommandCallback, this));
  commandSubscriber_.push_back(
    nodeHandle.subscribe(
      commandTopic_[1], 1,
      &MonstertruckSteerDriveController::ackermannDriveCommandCallback, this));

  // initialize position and velocity of wheels
  brake();
  alignWheels();

  return true;
}

void MonstertruckSteerDriveController::update(
  const ros::Time& time, const ros::Duration& period)
{
  // if last received command timed out, brake
  if ((time - steerDriveCommand_.stamp).toSec() > commandTimeout_)
  {
    brake();
    return;
  }

  // write velocity commands to the wheel joints
  leftWheelJoints_[0].setCommand(steerDriveCommand_.leftVelocity);
  leftWheelJoints_[1].setCommand(steerDriveCommand_.leftVelocity);
  rightWheelJoints_[0].setCommand(steerDriveCommand_.rightVelocity);
  rightWheelJoints_[1].setCommand(steerDriveCommand_.rightVelocity);

  // write steer commands to the steer joints
  leftSteerJoints_[0].setCommand(steerDriveCommand_.leftSteerAngle);
  leftSteerJoints_[1].setCommand(-steerDriveCommand_.leftSteerAngle);
  rightSteerJoints_[0].setCommand(steerDriveCommand_.rightSteerAngle);
  rightSteerJoints_[1].setCommand(-steerDriveCommand_.rightSteerAngle);
}

void MonstertruckSteerDriveController::starting(const ros::Time& time)
{
  brake();
  alignWheels();
}

void MonstertruckSteerDriveController::stopping(const ros::Time& time)
{
  brake();
  alignWheels();
}

void MonstertruckSteerDriveController::brake()
{
  for (int ii = 0; ii < 2; ii++)
  {
    leftWheelJoints_[ii].setCommand(0.0);
    rightWheelJoints_[ii].setCommand(0.0);
  }
}

void MonstertruckSteerDriveController::alignWheels()
{
  for (int ii = 0; ii < 2; ii++)
  {
    leftSteerJoints_[ii].setCommand(0.0);
    rightSteerJoints_[ii].setCommand(0.0);
  }
}

void MonstertruckSteerDriveController::twistCommandCallback(
  const geometry_msgs::TwistConstPtr& msg)
{
  if (!isRunning())
  {
    ROS_ERROR(
      "[MonstertruckSteerDriveController] Controller not accepting commands!!!");
    return;
  }

  if (fabs(msg->linear.y) < 0.01)
  {  // counter steering mode
    double turningRadius = msg->linear.x / msg->angular.z;
    computeCounterSteerCommand(msg->linear.x, turningRadius);
  }
  else
  {  // parallel steering mode
    computeParallelSteerCommand(msg->linear.x, msg->linear.y);
  }
}

void MonstertruckSteerDriveController::ackermannDriveCommandCallback(
  const ackermann_msgs::AckermannDriveConstPtr& msg)
{
  if (!isRunning())
  {
    ROS_ERROR(
      "[MonstertruckSteerDriveController] Controller not accepting commands!!!");
    return;
  }

  // compute steer drive command
  double turningRadius = wheelbase_ / tan(msg->steering_angle);
  computeCounterSteerCommand(msg->speed, turningRadius);
}

void MonstertruckSteerDriveController::computeCounterSteerCommand(
  double speed, double turningRadius)
{
  if (fabs(turningRadius) < minTurningRadius_)
  {
    brake();
    alignWheels();
    ROS_ERROR("[MonstertruckSteerDriveController] Invalid command!");
    return;
  }
  else if (isnan(turningRadius))
  {
    brake();
    alignWheels();
    return;
  }

  // inner: the side of the robot closest to the center of rotation
  // outer: the side of the robot farthest from the center of rotation

  double deltaInner;  // inner wheel steer angle
  double deltaOuter;  // outer wheel steer angle
  double deltaMean;  // virtual middle wheel steer angle

  double innerVel;  // inner wheel velocity
  double outerVel;  // outer wheel velocity

  double R = turningRadius;  // mobile base trajectory turning radius

  deltaMean = atan(wheelbase_ / 2 / R);
  deltaInner = fabs(deltaMean);
  deltaOuter = fabs(deltaMean);

  if (!isinf(R) && speed != 0)
  {
    double innerR = wheelbase_ / 2 / sin(atan(wheelbase_ / 2 /
        (fabs(R) - track_ / 2)));
    double vInner4WS = 2 * speed * fabs(tan(deltaMean) * innerR / wheelbase_);
    double innerWheelSlipAngle = cos(atan(fabs(
          wheelbase_ / 2 / (fabs(R) - track_/2) - deltaInner)));
    // inner-to-the-turn wheels
    innerVel = vInner4WS / innerWheelSlipAngle / wheelRadius_;

    double outerR = wheelbase_ / 2 / sin(atan(wheelbase_ / 2 /
        (fabs(R) + track_ / 2)));
    double vOuter4WS = 2 * speed * fabs(tan(deltaMean) * outerR / wheelbase_);
    double outerWheelSlipAngle = cos(atan(fabs(
          wheelbase_ / 2 / (fabs(R)+ track_/2) - deltaOuter)));
    // outer-to-the-turn wheels
    outerVel = vOuter4WS / outerWheelSlipAngle / wheelRadius_;
  }
  else
  {
    innerVel = speed / wheelRadius_;
    outerVel = speed / wheelRadius_;
  }

  steerDriveCommand_.leftVelocity = (deltaMean > 0) ? innerVel : outerVel;
  steerDriveCommand_.rightVelocity = (deltaMean > 0) ? outerVel : innerVel;
  steerDriveCommand_.leftSteerAngle =
    (deltaMean > 0) ? deltaInner : -deltaOuter;
  steerDriveCommand_.rightSteerAngle =
    (deltaMean > 0) ? deltaOuter : -deltaInner;
  steerDriveCommand_.stamp = ros::Time::now();
}

void MonstertruckSteerDriveController::computeParallelSteerCommand(
  double vx,  double vy)
{
  double betaAngle = atan(vx / vy);  // the sideslip angle of the vehicle
  double velocity = sqrt(vx * vx + vy * vy);  // the velocity of the robot

  if  (fabs(betaAngle) < maxSteerAngle_)
  {
    steerDriveCommand_.leftVelocity = velocity;
    steerDriveCommand_.rightVelocity = velocity;
    steerDriveCommand_.leftSteerAngle = betaAngle;
    steerDriveCommand_.rightSteerAngle = betaAngle;
    steerDriveCommand_.stamp = ros::Time::now();
  }
}

void MonstertruckSteerDriveController::loadJointNamesAndParameters(
  ros::NodeHandle& nodeHandle)
{
  // load robot parameters from the parameter server
  ROS_ASSERT_MSG(
    nodeHandle.getParam("wheel_radius", wheelRadius_),
    "wheel_radius not set in parameter server!");
  ROS_ASSERT_MSG(
    nodeHandle.getParam("wheelbase", wheelbase_),
    "wheelbase not set in parameter server!");
  ROS_ASSERT_MSG(
    nodeHandle.getParam("track", track_),
    "track not set in parameter server!");
  ROS_ASSERT_MSG(
    nodeHandle.getParam("max_steer_angle", maxSteerAngle_),
    "max_steer_angle not set in parameter server");

  std::string commandTopic;
  ROS_ASSERT_MSG(
    nodeHandle.getParam("twist_command_topic", commandTopic),
    "command_topic not set in parameter server!");
  commandTopic_.push_back(commandTopic);

  ROS_ASSERT_MSG(
    nodeHandle.getParam("ackermann_command_topic", commandTopic),
    "command_topic not set in parameter server!");
  commandTopic_.push_back(commandTopic);

  std::string jointName;

  // load wheel joint names from the parameter server
  ROS_ASSERT_MSG(
    nodeHandle.getParam("wheel_joints/left/front", jointName),
    "wheel_joints/left/front not set in parameter server!");
  leftWheelJointNames_.push_back(jointName);

  ROS_ASSERT_MSG(
    nodeHandle.getParam("wheel_joints/left/rear", jointName),
    "wheel_joints/left/rear not set in parameter server!");
  leftWheelJointNames_.push_back(jointName);

  ROS_ASSERT_MSG(
    nodeHandle.getParam("wheel_joints/right/front", jointName),
    "wheel_joints/right/front not set in parameter server!");
  rightWheelJointNames_.push_back(jointName);

  ROS_ASSERT_MSG(
    nodeHandle.getParam("wheel_joints/right/rear", jointName),
    "wheel_joints/right/rear not set in parameter server!");
  rightWheelJointNames_.push_back(jointName);

  // load wheel steer joints from the parameter server
  ROS_ASSERT_MSG(
    nodeHandle.getParam("steer_joints/left/front", jointName),
    "steer_joints/left/front not set in parameter server!");
  leftSteerJointNames_.push_back(jointName);

  ROS_ASSERT_MSG(
    nodeHandle.getParam("steer_joints/left/rear", jointName),
    "wheel_joints/left/rear not set in parameter server!");
  leftSteerJointNames_.push_back(jointName);

  ROS_ASSERT_MSG(
    nodeHandle.getParam("steer_joints/right/front", jointName),
    "wheel_joints/right/front set in parameter server!");
  rightSteerJointNames_.push_back(jointName);

  ROS_ASSERT_MSG(
    nodeHandle.getParam("steer_joints/right/rear", jointName),
    "wheel_joints/right/rear not set in parameter server!");
  rightSteerJointNames_.push_back(jointName);
}

}  // namespace monstertruck_steer_drive_controller
