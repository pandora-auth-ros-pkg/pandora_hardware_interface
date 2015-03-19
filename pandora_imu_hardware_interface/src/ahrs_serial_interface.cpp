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
* Author: Chris Zalidis
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
    AbstractImuSerialInterface(device, speed, timeout)
  {
  }

  void AhrsSerialInterface::init()
  {
    if (serialPtr_ == NULL)
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
      throw std::logic_error("Init called twice!!");
    }
  }

  void AhrsSerialInterface::read()
  {
    if (serialPtr_ == NULL)
      ROS_ERROR("read() called before init()!");
    else if (!serialPtr_->isOpen())
      ROS_ERROR("Port not open");

    // command: numBytes, command, crc
    unsigned char command[] = {0x00, 0x05, kGetData, 0x00, 0x00};
    // calculate crc of command and store it in the 4th & 5th byte of command
    uint16_t crc_code = calcCrc((unsigned char*)command,
      sizeof(command)/sizeof(unsigned char));
    // write command kGetData to receive yaw,pitc,roll measurements
    serialPtr_->write(command, 5);

    std::string buffer;
    // read data packet
    serialPtr_->readline(buffer);

    /*
    for (int ii = 0; ii < buffer.size(); ii++)
    {
      ROS_INFO(
        "PacketByte[%d]: [%02x]", ii, *(unsigned char*)(buffer.c_str()+ii)); 
    }
    */
    // extract size of packet from packet
    uint16_t bufferSize =
      static_cast<uint16_t>(buffer.at(0) << 8 | buffer.at(1));

    // calculate CRC of data packet
    crc_code = calcCrc((unsigned char*) buffer.c_str(), buffer.size());

    // check data packet
    if (bufferSize == buffer.size())
      if (check(buffer, crc_code))
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


  uint16_t AhrsSerialInterface::calcCrc(unsigned char* data, size_t data_size)
  {
    uint16_t crc = 0;
    for (int ii = 0; ii < data_size-2; ii++)
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
    data[data_size-2] = (crc & 0xFF00) >> 8;
    data[data_size-1] = crc & 0xFF;

    return crc;
  }


  void AhrsSerialInterface::parse(const std::string& packet)
  {
    char codes[3] = {YAW_CODE, PITCH_CODE, ROLL_CODE};
    size_t start = std::string::npos;
    bool found = true;

    start = packet.find_first_of(YAW_CODE, 0) + 2;
    std::string subPacket = packet.substr(start, 15);

    for (int ii = 0; ii < 3; ii++)
    {
      start = subPacket.find_first_of(codes[ii]);
      if (start == std::string::npos){
        ROS_ERROR("Failed to locate code in bytestream");
        found = false;
      }
      else
      {
        std::reverse_copy(
          subPacket.begin() + start + 1,
          subPacket.begin() + start + 1 + sizeof(float),
          &rotations.charBuffer[ii * sizeof(float)]);
      }
    }
    if (found &&
      rotations.data[0] <= 360.0 && rotations.data[0] >= -360.0 &&
      rotations.data[1] <= 360.0 && rotations.data[1] >= -360.0 &&
      rotations.data[2] <= 360.0 && rotations.data[2] >= -360.0)
    {
      yaw_ = rotations.data[0];
      pitch_ = rotations.data[1];
      roll_ = rotations.data[2];
//      ROS_INFO("yaw[%x], pitch[%x], roll[%x]", *(int *)&yaw_, *(int *)&pitch_, *(int *)&roll_);
//      ROS_INFO("yaw[%f], pitch[%f], roll[%f]", yaw_, pitch_, roll_);
    }
  }
}  // namespace imu
}  // namespace pandora_hardware_interface
