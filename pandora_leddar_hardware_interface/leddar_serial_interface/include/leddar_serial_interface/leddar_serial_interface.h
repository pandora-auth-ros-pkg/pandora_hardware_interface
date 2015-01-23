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

#ifndef LEDDAR_SERIAL_INTERFACE_LEDDAR_SERIAL_INTERFACE_H
#define LEDDAR_SERIAL_INTERFACE_LEDDAR_SERIAL_INTERFACE_H

#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include "leddar_serial_interface/Leddar.h"

namespace pandora_hardware_interface
{
namespace leddar
{
  class LeddarSerialInterface : private boost::noncopyable
  {
    public:
      LeddarSerialInterface(
        std::string device,
        std::string port_number,
        int address);
      ~LeddarSerialInterface();
      void init();
      void read();
      LtAcquisition* getLAcquisition()
      {
        return lAcquisition_;
      }

    private:
      std::string device_;
      std::string port_name_;  // ttyUSB*
      int address_;  // 1-255
      LtAcquisition *lAcquisition_;
  };
}  // namespace leddar
}  // namespace pandora_hardware_interface
#endif  // LEDDAR_SERIAL_INTERFACE_LEDDAR_SERIAL_INTERFACE_H
