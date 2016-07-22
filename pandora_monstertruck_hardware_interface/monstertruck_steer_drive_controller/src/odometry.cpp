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
 * Author: George Kouros
 *********************************************************************/

/*
 * Odometry Calculator for Bi-Steerable robots based on the original
 * implementation for differential drive robots from ros_controllers and written
 * by Luca Marchionni, Bence Magyar, Enrique Fernández, Paul Mathieu
 */

#include "monstertruck_steer_drive_controller/odometry.h"
#include <math.h>

namespace monstertruck_steer_drive_controller
{

  Odometry::Odometry()
  : stamp_(0.0)
  , wheelRadius_(0.75)
  , wheelbase_(0.32)
  , track_(0.26)
  {
  }


  Odometry::~Odometry()
  {
  }


  void Odometry::init(const ros::Time& time)
  {
    stamp_ = time;
    pose_.orientation.w = 1.0;
  }


  bool Odometry::update(
    const geometry_msgs::Twist& newTwist, const ros::Time& time)
  {
    double dt = (time - stamp_).toSec();
    stamp_ = time;
    integrate(newTwist, dt);
  }


  void Odometry::setWheelParams(
    double wheelRadius, double wheelbase, double track)
  {
    wheelRadius_ = wheelRadius;
    wheelbase_ = wheelbase;
    track_ = track;
  }


  void Odometry::integrate(const geometry_msgs::Twist& newTwist, double dt)
  {
    prevTwist_ = twist_;
    twist_ = newTwist;
    prevPose_ = pose_;

    double ds = hypot(twist_.linear.x, twist_.linear.y) * dt;
    double dth = twist_.angular.z * dt;
    double beta = atan2(twist_.linear.y, twist_.linear.x);
    double prevBeta = atan2(prevTwist_.linear.y, prevTwist_.linear.x);
    double dbeta = beta - prevBeta;

    pose_.position.x += ds * cos(tf::getYaw(prevPose_.orientation) + dth/2
      + prevBeta + dbeta/2);

    pose_.position.y += ds * sin(tf::getYaw(prevPose_.orientation) + dth/2
      + prevBeta + dbeta/2);

    pose_.orientation = tf::createQuaternionMsgFromYaw(
      tf::getYaw(prevPose_.orientation) + dth);
  }

}  // namespace monstertruck_steer_drive_controller
