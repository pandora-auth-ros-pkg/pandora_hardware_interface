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
  , wheelRadius_(0.0)
  , wheelbase_(0.0)
  , rearAxleFactor_(0.0)
  {
  }


  Odometry::~Odometry()
  {
  }


  void Odometry::init(const ros::Time& time, double wheelRadius,
    double wheelbase, double rearAxleFactor)
  {
    stamp_ = time;
    wheelRadius_ = wheelRadius;
    wheelbase_ = wheelbase;
    rearAxleFactor_ = rearAxleFactor;

    pose_.orientation.w = 1.0;
  }


  bool Odometry::update(const ros::Time& time, double velocity,
    double frontSteeringAngle, double rearSteeringAngle)
  {
    double dt = (time - stamp_).toSec();
    stamp_ = time;
    integrate(dt, velocity, frontSteeringAngle, rearSteeringAngle);
  }


  void Odometry::integrate(double dt, double velocity,
    double frontSteeringAngle, double rearSteeringAngle)
  {
    prevTwist_ = twist_;
    prevPose_ = pose_;

    double beta = atan((rearAxleFactor_ * tan(frontSteeringAngle)
      + (1 - rearAxleFactor_) * tan(rearSteeringAngle)) / wheelbase_);

    twist_.linear.x = velocity / sqrt(1 + pow(tan(beta), 2));
    twist_.linear.y = twist_.linear.x * tan(beta);
    twist_.angular.z = velocity * cos(beta)
      * (tan(frontSteeringAngle) - tan(rearSteeringAngle)) / wheelbase_;

    double r = wheelbase_ / fabs(cos(beta)
      * (tan(frontSteeringAngle) - tan(rearSteeringAngle)));

    double ds = velocity * dt;
    double dth = twist_.angular.z * dt;

    double prevBeta = atan(prevTwist_.linear.y / prevTwist_.linear.x);
    prevBeta = isnan(prevBeta) ? 0.0 : prevBeta;
    double dbeta = beta - prevBeta;

    // my equations
    pose_.position.x += velocity * (sin(tf::getYaw(prevPose_.orientation) + dth + beta + dbeta)
      - sin(tf::getYaw(prevPose_.orientation) + beta)) / (twist_.angular.z + dbeta/dt);
    pose_.position.y += velocity * (-cos(tf::getYaw(prevPose_.orientation) + dth + prevBeta + dbeta)
      + cos(tf::getYaw(prevPose_.orientation) + prevBeta)) / (twist_.angular.z + dbeta/dt);

    /*
     * Equations from paper "Automated Odometry Self-Calibration for Car-Like Robots
     * with Four Wheel Steering (2012)"
     */
    // pose_.position.x += ds * cos(tf::getYaw(prevPose_.orientation) + dth/2
      // + prevBeta + dbeta/2);
    // pose_.position.y += ds * sin(tf::getYaw(prevPose_.orientation) + dth/2
      // + prevBeta + dbeta/2);


    pose_.orientation = tf::createQuaternionMsgFromYaw(
      tf::getYaw(prevPose_.orientation) + dth);
  }

}  // namespace monstertruck_steer_drive_controller
