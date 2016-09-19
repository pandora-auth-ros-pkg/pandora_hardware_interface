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

#include <tf/transform_datatypes.h>
#include <boost/assign.hpp>
#include <cmath>

namespace
{
  double sgn(double x)
  {
    return (x >= 0.0) ? 1.0 : -1.0;
  }
}


namespace monstertruck_steer_drive_controller
{

MonstertruckSteerDriveController::MonstertruckSteerDriveController()
  : cmdTimeout_(0.5)
  , ideal4WS_(true)
  , wheelRadius_(0.0)
  , wheelbase_(0.0)
  , rearAxleFactor_(0.0)
  , track_(0.0)
  , ctrlName_("monstertruck_steer_drive_controller")
  , enableOdomTf_(true)
{
}


bool MonstertruckSteerDriveController::init(
  hardware_interface::VelocityJointInterface* velJointInterface,
  hardware_interface::PositionJointInterface* posJointInterface,
  ros::NodeHandle& nh)
{
  // load joint names and parameters from the parameter server
  loadParams(nh);

  // load drive joint handles
  leftFrontDriveJoint_ =
    velJointInterface->getHandle(leftFrontDriveJointName_);
  leftRearDriveJoint_ =
    velJointInterface->getHandle(leftRearDriveJointName_);
  rightFrontDriveJoint_ =
    velJointInterface->getHandle(rightFrontDriveJointName_);
  rightRearDriveJoint_ =
    velJointInterface->getHandle(rightRearDriveJointName_);

  // load steer joint handles
  leftFrontSteerJoint_ =
    posJointInterface->getHandle(leftFrontSteerJointName_);
  leftRearSteerJoint_ =
    posJointInterface->getHandle(leftRearSteerJointName_);
  rightFrontSteerJoint_ =
    posJointInterface->getHandle(rightFrontSteerJointName_);
  rightRearSteerJoint_ =
    posJointInterface->getHandle(rightRearSteerJointName_);

  // initialize twist and ackermann cmd subscribers
  twistCmdSub_ = nh.subscribe("/cmd_vel", 1,
    &MonstertruckSteerDriveController::twistCommandCallback, this);
  ackCmdSub_ = nh.subscribe("/ackermann_cmd", 1,
      &MonstertruckSteerDriveController::ackermannDriveCommandCallback, this);

  return true;
}


void MonstertruckSteerDriveController::update(
  const ros::Time& time, const ros::Duration& period)
{
  // get steering angles feedback
  double frontSteeringAngle = leftFrontSteerJoint_.getPosition() / 2
    + rightFrontSteerJoint_.getPosition() / 2;
  double rearSteeringAngle = leftRearSteerJoint_.getPosition() / 2
    + rightRearSteerJoint_.getPosition() / 2;
  double beta = atan((rearAxleFactor_ * tan(frontSteeringAngle)
    + (1-rearAxleFactor_) * tan(rearSteeringAngle)) / wheelbase_);

  double R = fabs(
    wheelbase_ / (tan(frontSteeringAngle) - tan(rearSteeringAngle)));

  // get velocities feedback
  double vf = (leftFrontDriveJoint_.getVelocity()
    + rightFrontDriveJoint_.getVelocity()) / 2 * wheelRadius_;
  double vr = (leftRearDriveJoint_.getVelocity()
    + rightRearDriveJoint_.getVelocity()) / 2 * wheelRadius_;
  double velocity = (vf * cos(frontSteeringAngle) + vr * cos(rearSteeringAngle))
    / 2 / cos(beta);

  // update odometry
  odometry_.update(time, velocity, frontSteeringAngle, rearSteeringAngle);

  // update odom msg and publish
  if (lastOdomPubTime_ + odomPubPeriod_ < time)
  {
    odomPub_->msg_.header.stamp = time;
    odometry_.getPose(odomPub_->msg_.pose.pose);
    odometry_.getTwist(odomPub_->msg_.twist.twist);
    odomPub_->unlockAndPublish();
  }

  // publish tf /odom frame
  if (enableOdomTf_)
  {
    geometry_msgs::Pose pose;
    odometry_.getPose(pose);
    geometry_msgs::TransformStamped& odomFrame = tfOdomPub_->msg_.transforms[0];
    odomFrame.header.stamp = time;
    odomFrame.transform.translation.x = pose.position.x;
    odomFrame.transform.translation.y = pose.position.y;
    odomFrame.transform.rotation = pose.orientation;
    tfOdomPub_->unlockAndPublish();
  }

  // if last received command timed out, brake
  if ((time - steerDriveCommand_.stamp).toSec() > cmdTimeout_)
    brake();

  // write velocity commands to the wheel joints
  leftFrontDriveJoint_.setCommand(steerDriveCommand_.leftFrontWheelVelocity);
  leftRearDriveJoint_.setCommand(steerDriveCommand_.leftRearWheelVelocity);
  rightFrontDriveJoint_.setCommand(steerDriveCommand_.rightFrontWheelVelocity);
  rightRearDriveJoint_.setCommand(steerDriveCommand_.rightRearWheelVelocity);

  // write steer commands to the steer joints
  leftFrontSteerJoint_.setCommand(steerDriveCommand_.leftFrontSteeringAngle);
  rightFrontSteerJoint_.setCommand(steerDriveCommand_.rightFrontSteeringAngle);
  leftRearSteerJoint_.setCommand(steerDriveCommand_.leftRearSteeringAngle);
  rightRearSteerJoint_.setCommand(steerDriveCommand_.rightRearSteeringAngle);
}


void MonstertruckSteerDriveController::starting(const ros::Time& time)
{
  // initialize command
  steerDriveCommand_.stamp = ros::Time::now();

  // initialize odometry
  odometry_.init(time, wheelRadius_, wheelbase_, rearAxleFactor_);
  lastOdomPubTime_ = time;
}


void MonstertruckSteerDriveController::stopping(const ros::Time& time)
{
  brake();
  alignWheels();
}


void MonstertruckSteerDriveController::brake()
{
  steerDriveCommand_.leftFrontWheelVelocity = 0.0;
  steerDriveCommand_.rightFrontWheelVelocity = 0.0;
  steerDriveCommand_.leftRearWheelVelocity = 0.0;
  steerDriveCommand_.rightRearWheelVelocity = 0.0;
}


void MonstertruckSteerDriveController::alignWheels()
{
  steerDriveCommand_.leftFrontSteeringAngle = 0.0;
  steerDriveCommand_.rightFrontSteeringAngle = 0.0;
  steerDriveCommand_.leftRearSteeringAngle = 0.0;
  steerDriveCommand_.rightRearSteeringAngle = 0.0;
}


void MonstertruckSteerDriveController::ackermannDriveCommandCallback(
  const ackermann_msgs::AckermannDriveConstPtr& msg)
{
  if (!isRunning())
  {
    ROS_WARN("[%s] Controller not accepting commands!!!",
      ctrlName_.c_str());
    return;
  }

  convertBase2JointCmds(msg->speed, msg->steering_angle);
}


void MonstertruckSteerDriveController::twistCommandCallback(
  const geometry_msgs::TwistConstPtr& msg)
{
  if (!isRunning())
  {
    ROS_WARN("[%s] Controller not accepting commands!!!",
      ctrlName_.c_str());
    return;
  }

  convertBase2JointCmds(msg->linear.x, msg->linear.y, msg->angular.z);
}


void MonstertruckSteerDriveController::convertBase2JointCmds(
  double speed, double steeringAngle)
{
  double fsa = steeringAngle;  // front steering angle
  double v = speed;  // base speed

  if (fabs(fsa) > maxSteeringAngle_)
  {
    ROS_WARN("[%s] Received invalid command (front_steering_angle=%f < "
      "max_steering_angle=%f)!",
      ctrlName_.c_str(), fsa, maxSteeringAngle_);
    return;
  }

  // calculate turning radius of base and wheels
  double r = fabs(wheelbase_ / tan(fsa));  // base turning radius (tr)
  double rif = hypot(r - track_ / 2, wheelbase_);  // inner front wheel tr
  double rof = hypot(r + track_ / 2, wheelbase_);  // outer front wheel tr
  double rir = r - track_ / 2;  // inner rear wheel tr
  double ror = r + track_ / 2;  // outer rear wheel tr

  // calculate ideal spped of wheels
  double vif = v * cos(fsa) * rif / r;  // ideal inner front wheel speed
  double vof = v * cos(fsa) * rof / r;  // ideal outer front wheel speed
  double vir = v * rir / r;  // inner rear wheel speed
  double vor = v * ror / r;  // outer rear wheel speed

  // calculate ideal ackermann angles
  double ifsa = atan(wheelbase_ / (r - track_ / 2));  // ideal inner front ackermann angle
  double ofsa = atan(wheelbase_ / (r + track_ / 2));  // ideal outer front ackermann angle

  // calculate actual speed of wheels based on left to right steering relation
  vif /= cos(fabs(fsa) - ifsa);  // actual left front wheel speed
  vof /= cos(fabs(fsa) - ofsa);  // actual right front wheel speed

  // in case of near 0 steering angle (infinite turning radius) -> linear motion
  if (fabs(fsa) < 1e-3)
    vif = vof = vir = vor = v;

  // convert velocities from m/s to rad/s
  vif /= wheelRadius_;
  vof /= wheelRadius_;
  vir /= wheelRadius_;
  vor /= wheelRadius_;

  steerDriveCommand_.leftFrontSteeringAngle = fsa;
  steerDriveCommand_.rightFrontSteeringAngle = fsa;
  steerDriveCommand_.leftRearSteeringAngle = 0.0;
  steerDriveCommand_.rightRearSteeringAngle = 0.0;
  steerDriveCommand_.leftFrontWheelVelocity = (fsa >= 0) ? vif : vof;
  steerDriveCommand_.leftRearWheelVelocity = (fsa >= 0) ? vir : vor;
  steerDriveCommand_.rightFrontWheelVelocity = (fsa >= 0) ? vof : vif;
  steerDriveCommand_.rightRearWheelVelocity = (fsa >= 0) ? vor : vir;
  steerDriveCommand_.stamp = ros::Time::now();
}


void MonstertruckSteerDriveController::convertBase2JointCmds(
  double linVelX, double linVelY, double angVel)
{
  // calculate base linear velocity on the plane
  double v = sgn(linVelX) * hypot(linVelX, linVelY);

  // robot parameters
  double w = track_;  // the track of the robot
  double lf = (1 - rearAxleFactor_) * wheelbase_;  // d(front_axle, CoG)
  double lr = rearAxleFactor_ * wheelbase_;  // d(rear_axle, CoG)

  // calculate sideslip angle and front and rear steering angles
  double beta = (fabs(linVelY) > 1e-3) ? atan(linVelY / linVelX) : 0.0;
  double fsa = atan(angVel * lf / v / cos(beta) + tan(beta));
  double rsa = atan(-angVel * lr / v / cos(beta) + tan(beta));

  // check if fsa or rsa is nan due to zero angular velocity
  fsa = isnan(fsa) ? 0.0 : fsa;
  rsa = isnan(rsa) ? 0.0 : rsa;

  if (fabs(beta) > maxSteeringAngle_)  // beta > beta_max
  {
    ROS_WARN("[%s] Received Invalid Command (beta=%f > beta_max=%f)",
      ctrlName_.c_str(), beta, maxSteeringAngle_);
    return;
  }

  if (fabs(fsa) > maxSteeringAngle_)  // beta > beta_max
  {
    ROS_WARN("[%s] Received Invalid Command (front_steering_angle=%f > "
      "max_steering_angle=%f)",
      ctrlName_.c_str(), fsa, maxSteeringAngle_);
    return;
  }

  if (fabs(rsa) > maxSteeringAngle_)  // beta > beta_max
  {
    ROS_WARN("[%s] Received Invalid Command (rear_steering_angle=%f > "
      "max_steering_angle=%f)",
      ctrlName_.c_str(), rsa, maxSteeringAngle_);
    return;
  }

  // calculate turning radius of CoG
  double r = fabs((lf + lr) / cos(beta) / (tan(fsa) - tan(rsa)));

  if (r < wheelbase_ / 2 / tan(maxSteeringAngle_))  // r < rmin
  {
    ROS_WARN("[%s] Received Invalid Command (R=%f < Rmin=%f)",
      ctrlName_.c_str(), r,
      wheelbase_ / 2 / tan(maxSteeringAngle_));
    return;
  }

  // calculate the turning radius of the front and rear axle midpoints
  double rf = r * cos(beta) / cos(fsa);
  double rr = r * cos(beta) / cos(rsa);

  // calculate velocity of base and the middle point of the front and rear axles
  double vf = v * rf / r;
  double vr = v * rr / r;

  vf = isnan(vf) ? v : vf;
  vr = isnan(vr) ? v : vr;

  // calculate the ideal 4WS steering angles for each wheel
  double ifsa = atan(sin(fsa) / (cos(fsa) - w / 2 / rf));
  double ofsa = atan(sin(fsa) / (cos(fsa) + w / 2 / rf));
  double irsa = atan(sin(rsa) / (cos(rsa) - w / 2 / rr));
  double orsa = atan(sin(rsa) / (cos(rsa) + w / 2 / rr));

  // calculate the ideal 4WS velocities for each wheel
  double vif = vf * (cos(fsa) - w / 2 / rf) / cos(ifsa);
  double vof = vf * (cos(fsa) + w / 2 / rf) / cos(ofsa);
  double vir = vr * (cos(rsa) - w / 2 / rr) / cos(irsa);
  double vor = vr * (cos(rsa) + w / 2 / rr) / cos(orsa);

  if (ideal4WS_)
  {
    if (fabs(ifsa) > maxSteeringAngle_)  // r < rmin
    {
      ROS_WARN("[%s] Received Invalid Command (inner front steering angle=%f > "
        "max_steering_angle=%f)", ctrlName_.c_str(), fabs(ifsa),
        maxSteeringAngle_);
      return;
    }
  }
  else
  {// calculate the actual velocities for each wheel in case of non ideal 4WS
    vif /= cos(fsa - ifsa);
    vof /= cos(fsa - ofsa);
    vir /= cos(fsa - irsa);
    vor /= cos(fsa - orsa);

    ifsa = ofsa = fsa;
    irsa = orsa = rsa;
  }

  if (fabs(fsa) < 1e-3)
    vif = vof = vir = vor = v;

  // convert velocities from m/s to rad/s
  vif /= wheelRadius_;
  vof /= wheelRadius_;
  vir /= wheelRadius_;
  vor /= wheelRadius_;

  // set the steer drive command
  steerDriveCommand_.leftFrontSteeringAngle = (fsa > 0.0) ? ifsa : ofsa;
  steerDriveCommand_.rightFrontSteeringAngle = (fsa > 0.0) ? ofsa : ifsa;
  steerDriveCommand_.leftRearSteeringAngle = (rsa > 0.0) ? orsa : irsa;
  steerDriveCommand_.rightRearSteeringAngle = (rsa > 0.0) ? irsa : orsa;
  steerDriveCommand_.leftFrontWheelVelocity = (fsa > 0.0) ? vif : vof;
  steerDriveCommand_.rightFrontWheelVelocity = (fsa > 0.0) ? vof : vif;
  steerDriveCommand_.leftRearWheelVelocity = (rsa > 0.0) ? vor : vir;
  steerDriveCommand_.rightRearWheelVelocity = (rsa > 0.0) ? vir : vor;
  steerDriveCommand_.stamp = ros::Time::now();
}


void MonstertruckSteerDriveController::loadParams(const ros::NodeHandle& nh)
{
  // load robot parameters from the parameter server
  ROS_ASSERT_MSG(nh.getParam("ideal_4ws", ideal4WS_),
    "ideal_4ws not set in parameter server");
  ROS_ASSERT_MSG(nh.getParam("wheel_radius", wheelRadius_),
    "wheel_radius not set in parameter server!");
  ROS_ASSERT_MSG(nh.getParam("wheelbase", wheelbase_),
    "wheelbase not set in parameter server!");
  ROS_ASSERT_MSG(nh.getParam("rear_axle_factor", rearAxleFactor_),
    "rear_axle_factor not set in parameter server!");
  ROS_ASSERT_MSG(nh.getParam("track", track_),
    "track not set in parameter server!");
  ROS_ASSERT_MSG(nh.getParam("max_steering_angle", maxSteeringAngle_),
    "max_steering_angle not set in parameter server");

  // load joint names from the parameter server
  ROS_ASSERT_MSG(
    nh.getParam("drive_joints/left/front", leftFrontDriveJointName_),
    "drive_joints/left/front not set in parameter server!");
  ROS_ASSERT_MSG(
    nh.getParam("drive_joints/left/rear", leftRearDriveJointName_),
    "drive_joints/left/rear not set in parameter server!");
  ROS_ASSERT_MSG(
    nh.getParam("drive_joints/right/front", rightFrontDriveJointName_),
    "drive_joints/right/front not set in parameter server!");
  ROS_ASSERT_MSG(
    nh.getParam("drive_joints/right/rear", rightRearDriveJointName_),
    "drive_joints/right/rear not set in parameter server!");

  // load steer joints from the parameter server
  ROS_ASSERT_MSG(
    nh.getParam("steer_joints/left/front", leftFrontSteerJointName_),
    "steer_joints/left/front not set in parameter server!");
  ROS_ASSERT_MSG(
    nh.getParam("steer_joints/left/rear", leftRearSteerJointName_),
    "steer_joints/left/rear not set in parameter server!");
  ROS_ASSERT_MSG(
    nh.getParam("steer_joints/right/front", rightFrontSteerJointName_),
    "steer_joints/right/front not set in parameter server!");
  ROS_ASSERT_MSG(
    nh.getParam("steer_joints/right/rear", rightRearSteerJointName_),
    "steer_joints/right/rear not set in parameter server!");

  // setup odometry msg
  std::vector<double> odomPoseCov, odomTwistCov;
  ROS_ASSERT_MSG(nh.getParam("odom_pose_covariance", odomPoseCov),
    "odom_pose_covariance not set in parameter server");
  ROS_ASSERT_MSG(nh.getParam("odom_twist_covariance", odomTwistCov),
    "odom_twist_covariance not set in parameter server");
  ROS_ASSERT_MSG(nh.getParam("enable_odom_tf", enableOdomTf_),
      "enable_odom_tf not set in parameter server");

  std::string odomTopic;
  ROS_ASSERT_MSG(nh.getParam("odom_topic", odomTopic),
      "odom_topic not set in parameter server");


  odomPub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(
      nh, odomTopic, 100));

  ROS_ASSERT_MSG(nh.getParam("base_frame_id", odomPub_->msg_.child_frame_id),
    "robot_frame_id not set in parameter server");

  odomPub_->msg_.header.frame_id = "odom";
  odomPub_->msg_.pose.covariance = boost::assign::list_of
    (odomPoseCov[0])  (0)  (0)  (0)  (0)  (0)
    (0)  (odomPoseCov[1])  (0)  (0)  (0)  (0)
    (0)  (0)  (odomPoseCov[2])  (0)  (0)  (0)
    (0)  (0)  (0)  (odomPoseCov[3])  (0)  (0)
    (0)  (0)  (0)  (0)  (odomPoseCov[4])  (0)
    (0)  (0)  (0)  (0)  (0)  (odomPoseCov[5]);
  odomPub_->msg_.twist.covariance = boost::assign::list_of
    (odomTwistCov[0])  (0)  (0)  (0)  (0)  (0)
    (0)  (odomTwistCov[1])  (0)  (0)  (0)  (0)
    (0)  (0)  (odomTwistCov[2])  (0)  (0)  (0)
    (0)  (0)  (0)  (odomTwistCov[3])  (0)  (0)
    (0)  (0)  (0)  (0)  (odomTwistCov[4])  (0)
    (0)  (0)  (0)  (0)  (0)  (odomTwistCov[5]);

    tfOdomPub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(nh,
        "/tf", 100));
    tfOdomPub_->msg_.transforms.resize(1);
    tfOdomPub_->msg_.transforms[0].transform.translation.z = 0.0;
    tfOdomPub_->msg_.transforms[0].child_frame_id =
      odomPub_->msg_.child_frame_id;
    tfOdomPub_->msg_.transforms[0].header.frame_id = "odom";
}

}  // namespace monstertruck_steer_drive_controller
