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
* Author:  Petros Evangelakos
*********************************************************************/
#ifndef PIXY_HARDWARE_INTERFACE_PIXY_HARDWARE_INTERFACE_H
#define PIXY_HARDWARE_INTERFACE_PIXY_HARDWARE_INTERFACE_H


#include "ros/ros.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <image_transport/image_transport.h>
#include <pixy.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>

namespace pandora_hardware_interface
{
  namespace pixy
  {
    class PixyHardwareInterface : public hardware_interface::RobotHW
    {
      public:
         explicit PixyHardwareInterface(
        ros::NodeHandle nodeHandle);
        ~PixyHardwareInterface();
        void read();
        void write();
        void readJointNameFromParamServer();
        void get_frame();
        sensor_msgs::Image img_;
        image_transport::Publisher image_pub;
        cv::Mat frame_;
      private:
        ros::NodeHandle nodeHandle_;
        image_transport::ImageTransport it_;
        hardware_interface::JointStateInterface jointStateInterface_;
        hardware_interface::PositionJointInterface positionJointInterface_;
        std::vector<std::string> jointNames_;
        double command_[2];
        double position_[2];
        double velocity_[2];
        double effort_[2];
    };
}  // namespace pixy
}  // namespace pandora_hardware_interface


#endif  // PIXY_HARDWARE_INTERFACE_PIXY_HARDWARE_INTERFACE_H
