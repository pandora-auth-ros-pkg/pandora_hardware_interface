/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
* Author:  George Kouros
*********************************************************************/
#ifndef LEDDAR_CONTROLLERS_LEDDAR_SENSOR_CONTROLLER_H
#define LEDDAR_CONTROLLERS_LEDDAR_SENSOR_CONTROLLER_H

#include <controller_interface/controller.h>
#include <leddar_hardware_interface/leddar_sensor_interface.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>

typedef boost::shared_ptr<realtime_tools::RealtimePublisher<
  sensor_msgs::LaserScan> > LeddarRealtimePublisher;

namespace pandora_hardware_interface
{
namespace leddar
{
  class LeddarSensorController :
    public controller_interface::Controller<LeddarSensorInterface>
  {
    public:
      LeddarSensorController();
      ~LeddarSensorController();
      virtual bool init(
        LeddarSensorInterface* leddarSensorInterface,
        ros::NodeHandle& rootNodeHandle,
        ros::NodeHandle& controllerNodeHandle);
      virtual void starting(const ros::Time& time);
      virtual void update(const ros::Time& time, const ros::Duration& period);
      virtual void stopping(const ros::Time& time);
      
    private:
      std::vector<LeddarSensorHandle> sensorHandles_;
      std::vector<LeddarRealtimePublisher> realtimePublishers_;
      std::vector<ros::Time> lastTimePublished_;
      double publishRate_;
  };
}  // namespace leddar
}  // namespace pandora_hardware_interface
#endif  // LEDDAR_CONTROLLERS_LEDDAR_SENSOR_CONTROLLER_H
