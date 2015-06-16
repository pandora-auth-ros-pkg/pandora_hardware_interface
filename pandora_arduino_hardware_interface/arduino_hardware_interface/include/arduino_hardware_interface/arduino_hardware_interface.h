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
* Author:  Evangelos Apostolidis
* Author:  George Kouros 
*********************************************************************/
#ifndef ARDUINO_HARDWARE_INTERFACE_ARDUINO_HARDWARE_INTERFACE_H
#define ARDUINO_HARDWARE_INTERFACE_ARDUINO_HARDWARE_INTERFACE_H


#include <boost/math/constants/constants.hpp>
#include "ros/ros.h"
#include "tf/tf.h"
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <arduino_hardware_interface/battery_interface.h>
#include <arduino_usb_interface/arduino_usb_interface.h>

namespace pandora_hardware_interface
{
namespace arduino
{
  /**
   @class arduinoHardwareInterface
   @brief Allows the controller manager to communicate with the arduino board
  **/
  class ArduinoHardwareInterface : public hardware_interface::RobotHW
  {
    public:
      /**
       @brief Default Constructor
       @details Establishes communication and registers thermal and CO2 
       interfaces
       @param nodeHandle [ros::NodeHandle] : handle of 
       arduino_hardware_interface_node
      **/
      explicit ArduinoHardwareInterface(
        ros::NodeHandle nodeHandle);

      /**
       @brief Default Constructor
       @details Frees arduinoUSBInterface instance
      **/
      ~ArduinoHardwareInterface();

      /**
       @brief Reads CO2 percentage and grideyes' values
       @return void
      **/
      void read();

    private:
 

     void registerBatteryInterface();

    private:
     ros::NodeHandle nodeHandle_;
     ArduinoUsbInterface* arduino_;


     /* battery interface */
     BatteryInterface batteryInterface_;
     std::vector<BatteryHandle::Data> batteryData_;
     std::vector<std::string> batteryName_;
     double* voltage_;
     char* batteryCode_;  // not stored in handle

  };
}  // namespace arduino
}  // namespace pandora_hardware_interface
#endif  // ARDUINO_HARDWARE_INTERFACE_ARDUINO_HARDWARE_INTERFACE_H
