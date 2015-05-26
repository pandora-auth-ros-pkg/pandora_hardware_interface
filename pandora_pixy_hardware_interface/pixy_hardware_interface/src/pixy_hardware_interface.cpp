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
#include <pixy_hardware_interface/pixy_hardware_interface.h>


namespace pandora_hardware_interface
{
  namespace pixy
  {

    PixyHardwareInterface::PixyHardwareInterface(
    ros::NodeHandle nodeHandle)
    :
      nodeHandle_(nodeHandle),
      it_(nodeHandle)
    {
      readJointNameFromParamServer();
      image_pub = it_.advertise("pixy_image", 1);
      int ret = pixy_init();
      if (ret != 0)
      {
        ROS_FATAL("PixyNode - %s - Failed to open with the USB error %d!",
        __FUNCTION__, ret);
        ROS_BREAK();
      }

    // connect and register the joint state interface
    for (int ii = 0; ii < 2; ii++)
    {
      position_[ii] = 0;
      velocity_[ii] = 0;
      effort_[ii] = 0;
      hardware_interface::JointStateHandle jointStateHandle(
        jointNames_[ii],
        &position_[ii],
        &velocity_[ii],
        &effort_[ii]);
      jointStateInterface_.registerHandle(jointStateHandle);
    }
      registerInterface(&jointStateInterface_);

    // connect and register the joint position interface
    for (int ii = 0; ii < 2; ii++)
    {
      hardware_interface::JointHandle jointPositionHandle(
        jointStateInterface_.getHandle(jointNames_[ii]),
        &command_[ii]);
      positionJointInterface_.registerHandle(jointPositionHandle);
    }
    registerInterface(&positionJointInterface_);
    pixy_rcs_set_position(0, 100);
    pixy_rcs_set_position(1, 100);
    }

    PixyHardwareInterface::~PixyHardwareInterface()
    {
    }

    void PixyHardwareInterface::read()
    {
      int feedback[2];
      feedback[0] = pixy_rcs_get_position(0);
      feedback[1] = pixy_rcs_get_position(1);
      position_[0] = static_cast<float>(feedback[0]);
      position_[1] = static_cast<float>(feedback[1]);
    }

    void PixyHardwareInterface::write()
    {
      uint16_t target[2];
      float cmd[2];
      cmd[0] = (command_[0] / 90.0) * 500 + 500;
      cmd[1] = (command_[1] / 90.0) * 500 + 500;
      target[0] = static_cast<uint16_t>(cmd[0]);
      target[1] = static_cast<uint16_t>(cmd[1]);
      if (target[0] >= 0 && target[0] <= 999   &&  target[1] >= 0 && target[1] <= 999)
      {
        pixy_rcs_set_position(0, target[0]);
        pixy_rcs_set_position(1, target[1]);
      }
      else
      {
        ROS_DEBUG_STREAM("Pixy servo command out of bounds");
      }
    }

    void PixyHardwareInterface::get_frame()
    {
      pixy_get_frame(frame_);
      fillImage(img_, "bgr8", frame_.rows, frame_.cols, frame_.channels() * frame_.cols, frame_.data);

      image_pub.publish(img_);
    }

    void PixyHardwareInterface::readJointNameFromParamServer()
  {
    std::string name;
    nodeHandle_.getParam(
      "pixy_pitch_joint",
      name);
    jointNames_.push_back(name);
    nodeHandle_.getParam(
      "pixy_yaw_joint",
      name);
    jointNames_.push_back(name);
  }

}  // namespace pixy
}  // namespace pandora_hardware_interface
