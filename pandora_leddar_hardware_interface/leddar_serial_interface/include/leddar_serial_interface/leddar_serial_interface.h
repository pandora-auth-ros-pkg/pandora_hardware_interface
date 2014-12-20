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
<<<<<<< Updated upstream:pandora_leddar_hardware_interface/leddar_serial_interface/include/leddar_serial_interface/leddar_serial_interface.h
=======
#ifndef XMEGA_HARDWARE_INTERFACE_ENCODER_SENSOR_INTERFACE_H
#define XMEGA_HARDWARE_INTERFACE_ENCODER_SENSOR_INTERFACE_H
>>>>>>> Stashed changes:pandora_xmega_hardware_interface/xmega_hardware_interface/include/xmega_hardware_interface/encoder_sensor_interface.h

#ifndef PANDORA_LEDDAR_HARDWARE_INTERFACE_LEDDAR_SERIAL_INTERFACE_H
#define PANDORA_LEDDAR_HARDWARE_INTERFACE_LEDDAR_SERIAL_INTERFACE_H

#include <stdio.h>
#include <serial/serial.h>
#include <ros/ros.h>
#include "LeddarC.h"
#include "LeddarProperties.h"

#define NUM_OF_DETECTIONS 16

namespace pandora_hardware_interface
{
namespace leddar
{
  class LeddarSerialInterface() : private boost::noncopyable
  {
    public:
<<<<<<< Updated upstream:pandora_leddar_hardware_interface/leddar_serial_interface/include/leddar_serial_interface/leddar_serial_interface.h
      LeddarSerialInterface(
          const std::string& device,
          int speed,
          int timeout);
          
      ~LeddarSerialInterface();
      
      void init();
      void dataCallback();
      void read();
      void readConfiguration();
      void writeConfiguration();
      
    private:
      const std::string device_;
      const int speed_;
      const int timeout_;
      LeddarHandle leddarHandle_;
      LdDetections lDetections[num_detections]
=======
      struct Data
      {
        Data()
        {
        }

        std::string name;
        double* degrees; 
      };
    
      EncoderSensorHandle(const Data& data = Data())
        :  name_(data.name),
           degrees_(data.degrees)
      {
      }
      
      ~EncoderSensorHandle()
      {
      }

      inline std::string getName() const
      {
        return name_;
      }
      
      inline const double* getDegrees() const
      {
        return degrees_;
      }
       
    private:
      std::string name_;
      double* degrees_;
>>>>>>> Stashed changes:pandora_xmega_hardware_interface/xmega_hardware_interface/include/xmega_hardware_interface/encoder_sensor_interface.h
  };

} // namespace leddar
} // namespace pandora_hardware_interface
#endif // PANDORA_LEDDAR_HARDWARE_INTERFACE_LEDDAR_SERIAL_INTERFACE_H

<<<<<<< Updated upstream:pandora_leddar_hardware_interface/leddar_serial_interface/include/leddar_serial_interface/leddar_serial_interface.h
=======
#endif // XMEGA_HARDWARE_INTERFACE_ENCODER_SENSOR_INTERFACE_H
>>>>>>> Stashed changes:pandora_xmega_hardware_interface/xmega_hardware_interface/include/xmega_hardware_interface/encoder_sensor_interface.h
