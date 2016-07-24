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

#ifndef MONSTERTRUCK_STEER_DRIVE_CONTROLLER_ODOMETRY_H
#define MONSTERTRUCK_STEER_DRIVE_CONTROLLER_ODOMETRY_H

#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

namespace monstertruck_steer_drive_controller
{

  class Odometry
  {
    public:

      Odometry();

      ~Odometry();

      void init(const ros::Time& time, double wheelRadius_, double wheelbase_,
        double rearAxleFactor_);

      bool update(const ros::Time& time, double velocity,
        double frontSteeringAngle, double rearSteeringAngle);


      double getX() {return pose_.position.x;}
      double getY() {return pose_.position.y;}
      double getTh() {return tf::getYaw(pose_.orientation);}
      double getLinVelX() {return twist_.linear.x;}
      double getLinVelY() {return twist_.linear.y;}
      double getAngVel() {return twist_.angular.z;}
      void getPose(geometry_msgs::Pose& pose) {pose = pose_;}
      void getTwist(geometry_msgs::Twist& twist) {twist = twist_;}


    private:

      void integrate(double dt, double velocity, double frontSteeringAngle,
        double rearSteeringAngle);

      ros::Time stamp_;

      geometry_msgs::Pose pose_;
      geometry_msgs::Pose prevPose_;

      geometry_msgs::Twist twist_;
      geometry_msgs::Twist prevTwist_;

      double wheelRadius_;
      double wheelbase_;
      double rearAxleFactor_;
  };

}  // namespace monstertruck_steer_drive_controller

#endif  // MONSTERTRUCK_STEER_DRIVE_CONTROLLER_ODOMETRY_H
