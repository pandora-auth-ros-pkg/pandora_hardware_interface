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
* Author: George Kouros
*********************************************************************/

#ifndef PANDORA_IMU_HARDWARE_INTERFACE_AHRS_SERIAL_INTERFACE_H
#define PANDORA_IMU_HARDWARE_INTERFACE_AHRS_SERIAL_INTERFACE_H

#include <boost/utility.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include "pandora_imu_hardware_interface/abstract_imu_serial_interface.h"

#define kGetData 0x04
#define YAW_CODE 0x05
#define PITCH_CODE 0x18
#define ROLL_CODE 0x19

namespace pandora_hardware_interface
{
namespace imu
{
  /**
   @class AhrsSerialInterface
   @brief Class used for serial communication with a Trax AHRS
  **/
  class AhrsSerialInterface : public AbstractImuSerialInterface
  {
   public:
    /**
     @brief Default Constructor
     @param device [std::string &] : AHRS device com port name
     @param speed [int] : Serial communication speed (baud rate)
     @param timeout [int] : Connection response timeout
    **/
    AhrsSerialInterface(
      const std::string& device,
      int speed,
      int timeout);

    /**
     @brief Establishes serial communication
     @return void
    **/
    void init();

    /**
     @brief Reads yaw, pitch and roll from the Trax AHRS
     @details Init must be called first to establish serial communication
     @return void
    **/
    void read();

    union Rotations
    {
      char charBuffer[12];
      float data[3];  // rotationData = {yaw,pitch,roll}
    };

   private:
    /**
     @brief Extract yaw, pitch, roll from packet
     @param packet [std::string&] : packet containing the raw ahrs data
     @return void
    **/
    void parse(const std::string& packet);

    /**
     @brief Check size of latest received data packet
     @return bool
    **/
    bool check(const std::string& packet, int crc);

    /**
     @brief Returns the crc of a byte stream using the xmodem crc algorithm
     @details The data_size must equal data.size-2
     @return uint16_t crc
    **/
    uint16_t calcCrc(unsigned char* data, size_t data_size);

   private:
    Rotations rotations;
    const boost::regex regex_;  //!< expression used to calculate yaw,pitch,roll
  };
}  // namespace imu
}  // namespace pandora_hardware_interface

#endif  // PANDORA_IMU_HARDWARE_INTERFACE_AHRS_SERIAL_INTERFACE_H
