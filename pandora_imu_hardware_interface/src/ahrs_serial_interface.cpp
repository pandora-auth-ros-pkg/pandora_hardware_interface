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

#include <pandora_imu_hardware_interface/ahrs_serial_interface.h>

namespace pandora_hardware_interface
{
namespace imu
{
  AhrsSerialInterface::AhrsSerialInterface(
    const std::string& device,
    int speed,
    int timeout)
  :
    AbstractImuSerialInterface(device, speed, timeout),
    regex_("^.+\x05(.{4})\x18(.{4})\x19(.{4}).+$", boost::regex::extended)
  {
  }


  void AhrsSerialInterface::init()
  {
    if (serialPtr_ == NULL || !serialPtr_->isOpen())
    {
      try
      {
        serialPtr_.reset(
          new serial::Serial(
            device_,
            speed_,
            serial::Timeout::simpleTimeout(timeout_)));
      }
      catch (serial::IOException& ex)
      {
        ROS_FATAL("[Trax AHRS] Cannot open port!!");
        ROS_FATAL("%s", ex.what());
        exit(-1);
      }
    }
    else
    {
      ROS_ERROR("Init called twice!!");
    }

    // compose set_endianness command
    unsigned char command[7] =
      {0x00, 0x07, kSetConfig, kBigEndian, LITTLE_ENDIAN_CODE, 0x00, 0x00};

    // calculate crc of command and store it in 5th and 6th bytes
    uint16_t crcCode = calcCrc(
      (unsigned char*) command, sizeof(command)/sizeof(unsigned char), true);

    // write set_endianness to little command to device
    serialPtr_->write(command, sizeof(command)/sizeof(unsigned char));
  }


  void AhrsSerialInterface::read()
  {
    if (serialPtr_ == NULL){
      ROS_ERROR("read() called before init().  Calling init() now...");
      init();
    }

    uint16_t crcCode;
    std::string buffer;

    serialPtr_->flush();  // flush IO buffer

    // Get yawm, pitch, roll measurements
    // command: numBytes(uint16), write command(uint8), crc(uint16)
    unsigned char command[5] = {0x00, 0x05, kGetData, 0x00, 0x00};
    // calculate crc of command and store it in the 4th & 5th byte of command
    crcCode = calcCrc(
      (unsigned char*)command, sizeof(command)/sizeof(unsigned char), true);
    // write command kGetData to receive yaw,pitc,roll measurements
    serialPtr_->write(command, sizeof(command)/sizeof(unsigned char));

    // read data packet
    serialPtr_->readline(buffer);

    // extract size of packet from packet
    uint16_t bufferSize =
      static_cast<uint16_t>(buffer.at(0) << 8 | buffer.at(1));

    // calculate CRC of data packet
    crcCode = calcCrc((unsigned char*) buffer.c_str(), buffer.size(), false);

    // check data packet
    if (bufferSize == buffer.size())
      if ( check(buffer, crcCode) )
        parse(buffer);  // parse data packet
  }


  bool AhrsSerialInterface::check(const std::string& packet, int crc)
  {
    int end = packet.size();
    char packetCrc[2] = {packet.at(end-1), packet.at(end-2)};

    if (*reinterpret_cast<uint16_t*>(packetCrc) == crc)
      return true;
    else
      return false;
  }


  uint16_t AhrsSerialInterface::calcCrc(
    unsigned char* data, size_t dataSize, bool storeCrcInData)
  {
    uint16_t crc = 0;
    for (int ii = 0; ii < dataSize-2; ii++)
    {
      crc = crc ^ ((uint16_t)data[ii] << 8);
      for (int jj = 0; jj < 8; jj++)
      {
        if (crc & 0x8000)
          crc = (crc << 1) ^ 0x1021;
        else
          crc <<= 1;
      }
    }
    if (storeCrcInData)
    {
      data[dataSize-2] = (crc & 0xFF00) >> 8;
      data[dataSize-1] = crc & 0xFF;
    }
    return crc;
  }


  void AhrsSerialInterface::parse(const std::string& packet)
  {
    boost::match_results<std::string::const_iterator> data;

    if (boost::regex_match(packet, data, regex_))
    {
      yaw_ = *reinterpret_cast<float*>(&data[1].str().at(0));
      pitch_ = *reinterpret_cast<float*>(&data[2].str().at(0));
      roll_ = *reinterpret_cast<float*>(&data[3].str().at(0));

      // ROS_INFO("yaw[%x], pitch[%x], roll[%x]", *(int *)&yaw_, *(int *)&pitch_, *(int *)&roll_);
      // ROS_INFO("yaw[%f], pitch[%f], roll[%f]", yaw_, pitch_, roll_);
    }
    else
      ROS_ERROR("Did not match received packet to desirable pattern");
  }
}  // namespace imu
}  // namespace pandora_hardware_interface
