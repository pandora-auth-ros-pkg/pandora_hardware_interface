/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: George Kouros
*********************************************************************/

#ifndef LEDDAR_HARDWARE_INTERFACE_LEDDAR_HARDWARE_INTERFACE_H
#define LEDDAR_HARDWARE_INTERFACE_LEDDAR_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "leddar_serial_interface/leddar_serial_interface.h"
#include "leddar_hardware_interface/leddar_sensor_interface.h"

namespace pandora_hardware_interface
{
namespace leddar
{
  class LeddarHardwareInterface : public hardware_interface::RobotHW
  {
    public:
      explicit LeddarHardwareInterface(ros::NodeHandle nodeHandle);
      ~LeddarHardwareInterface();
      void read();

    private:
      void registerLeddarSensorInterface();

    private:
      ros::NodeHandle nodeHandle_;
      LeddarSerialInterface leddarSerialInterface_;
      LeddarSensorInterface leddarSensorInterface_;
      LeddarSensorHandle::Data leddarSensorData_;
      LtAcquisition* lAcquisition_;

      std::string name_;
      std::string frameId_;
      int* leddarDetectionCount_;
      float* leddarDistances_;
      float* leddarAmplitudes_;
  };
}  // namespace leddar
}  // namespace pandora_hardware_interface
#endif  // LEDDAR_HARDWARE_INTERFACE_LEDDAR_HARDWARE_INTERFACE_H
