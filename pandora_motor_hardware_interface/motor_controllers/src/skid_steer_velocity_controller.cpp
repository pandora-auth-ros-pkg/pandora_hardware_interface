/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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
 *********************************************************************/

/*
 * Author: Konstantinos Panayiotou   <klpanagi@gmail.com>
 * Author: Konstantinos Zisis        <zisikons@gmail.com>
 * Author: Elisabet Papadopoulou     <papaelisabet@gmail.com >
 */

#include "motor_controllers/skid_steer_velocity_controller.h"

namespace pandora_hardware_interface
{
namespace motor
{

  bool SkidSteerVelocityController::init(hardware_interface::VelocityJointInterface* hw,
                                                                  ros::NodeHandle &ns)
  {
    // Load Joints from HW Interface , load joint NAMES from YAML
    std::string left_front_wheel_joint_name, right_front_wheel_joint_name;
    std::string left_rear_wheel_joint_name, right_rear_wheel_joint_name;

    ROS_INFO("STARTING CONTROLLER");

    if (!ns.getParam("left_front_wheel", left_front_wheel_joint_name))
    {
      ROS_ERROR("Could not find left fron wheel joint name");
      return false;
    }
    if (!ns.getParam("right_front_wheel", right_front_wheel_joint_name ))
    {
      ROS_ERROR("Could not find joint name");
      return false;
    }
    if (!ns.getParam("left_rear_wheel", left_rear_wheel_joint_name ))
    {
      ROS_ERROR("Could not find joint name");
      return false;
    }
    if (!ns.getParam("right_rear_wheel", right_rear_wheel_joint_name))
    {
      ROS_ERROR("Could not find joint name");
      return false;
    }

    // Get joint Handles from hw interface
    left_front_wheel_joint_ = hw->getHandle(left_front_wheel_joint_name);
    right_front_wheel_joint_ = hw->getHandle(right_front_wheel_joint_name);
    left_rear_wheel_joint_ = hw->getHandle(left_rear_wheel_joint_name);
    right_rear_wheel_joint_ = hw->getHandle(right_rear_wheel_joint_name);

    // Subscirbe to cmd_vel
    command_listener_ = ns.subscribe("/cmd_vel",
                                       1,
                                       &SkidSteerVelocityController::commandCallback,
                                       this);

    ROS_INFO("Successfully Initiallized controller!");
    return true;
  }

  void SkidSteerVelocityController::update(const ros::Time& time, const ros::Duration& period)
  {
    // Update with latest cmd_vel commands
    double w = command_struct_.ang;
    double v = command_struct_.lin;
    double a = 1.5;
    double B = 0.35;
    double wheel_radius = 0.0975;

    // Compute wheels velocities:  (Equations pandora_skid_steering.pdf )
    const double vel_left  = (1/wheel_radius)*v-((a*B)/(2*wheel_radius))*w;
    const double vel_right = (1/wheel_radius)*v+((a*B)/(2*wheel_radius))*w;
    // BEWARE!! : invert axes !! (paper vs URDF)

    // Set Joint Commands
    // ROS_INFO("%f %f",vel_left,vel_right);

    left_front_wheel_joint_.setCommand(vel_left);
    left_rear_wheel_joint_.setCommand(vel_left);
    right_front_wheel_joint_.setCommand(vel_right);
    right_rear_wheel_joint_.setCommand(vel_right);
  }

  void SkidSteerVelocityController::commandCallback(const geometry_msgs::Twist& command)
  {
    // Update command struct
    command_struct_.ang   = command.angular.z;
    command_struct_.lin   = command.linear.x;
    command_struct_.stamp = ros::Time::now();
  }

}  // namespace motor
}  // namespace pandora_hardware_interface
