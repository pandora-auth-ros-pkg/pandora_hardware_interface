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
 * Author:  George Kouros <gkourosg@yahoo.gr>
 *********************************************************************/


#ifdef MOTOR_CONTROLLERS_FOUR_WHEEL_STEER_VELOCITY_CONTROLLER_H
#define MOTOR_CONTROLLERS_FOUR_WHEEL_STEER_VELOCITY_CONTROLLER_H

namespace pandora_hardware_interface
{
namespace motors
{
  class FourWheelSteerVelocityController
    : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
    public:
      FourWheelSteerVelocityController(){}

    /**
     * brief Initialize controller
     * param hw [hardware_interface::VelocityJointInterface*]: wheel velocity joint interface
     * param nodeHandle [ros:NodeHandle &]: controller node handle
     */
    bool init(
      hardware_interface::VelocityJointInterface* hw,
      ros::NodeHandle& nodeHandle);

    /**
     * brief Update controller command
     * param time [ros::Time& time]: current time
     * param period [ros::Time& time]: period elapsed since last update
     * return void
     */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
     * brief Start controller
     * param time [ros::Time&]: current time
     * return void
     */
    void starting(const ros::Time& time){}

    /**
     * brief Stop controller
     * param time [ros::Time&]: current time
     * return void
     */
    void stopping(const ros::Time& time){}

    /**
     * brief Callback for robot twist command msgs
     * command [const geometry_msgs::Twist&]: robot twist command
     * return void
     */
    void twistCommandCallback(const geometry_msgs::Twist& command);

   private:
    hardware_interface::JointHandle left_front_wheel_joint_;  //!< left front wheel joint handle
    hardware_interface::JointHandle right_front_wheel_joint_;  //!< right front wheel joint handle
    hardware_interface::JointHandle left_rear_wheel_joint_;  //!< left rear wheel joint handle
    hardware_interface::JointHandle right_rear_wheel_joint_;  //!< right rear wheel joint handle

    ros::Subscriber twist_listener_;  //!< twist command message subscriber

    struct VelocityCommand
    {
      // TODO(gkouros): addl motor angular velocity and front,rear steering angles ???
    };
    VelocityCommand velCmd_;  //!< velocity command struct

    double wheelRadius_;  //!< the radius of the wheels of the vehicle
    double wheelbase_;  //!< the wheelbase of the vehicle
    double track_;  //!< the track of the vehicle
  };

  PLUGINLIB_EXPORT_CLASS(
    pandora_hardware_interface::motor::FourWheelSteerVelocityController,
    controller_interface::ControllerBase);

}  // namespace motor
}  // namespace pandora_hardware_interface
#endif  // MOTOR_CONTROLLERS_FOUR_WHEEL_STEERING_VELOCITY_CONTROLLER_H
