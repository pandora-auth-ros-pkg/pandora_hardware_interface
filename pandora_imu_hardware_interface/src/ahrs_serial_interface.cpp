/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
    regex_(
      ".*\x05(.{4})\x18(.{4})\x19(.{4})"
      "\x15(.{4})\x16(.{4})\x17(.{4})"
      "\x4a(.{4})\x4b(.{4})\x4c(.{4}).*",
      boost::regex::extended)
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

    // configure endianess of data in ahrs packet
    char endianessCmd[3] = {
      K_SET_CONFIG,
      K_BIG_ENDIAN,
      K_FALSE};
    write(endianessCmd, 3);

    // configure packet composition of ahrs
    char pkgCompositionCmd[11] = {
      K_SET_DATA_COMPONENTS,
      0x09,
      K_HEADING,
      K_PITCH,
      K_ROLL,
      K_ACCEL_X,
      K_ACCEL_Y,
      K_ACCEL_Z,
      K_GYRO_X,
      K_GYRO_Y,
      K_GYRO_Z};
    write(pkgCompositionCmd, 11);

    // configure ahrs for continuous mode
    char setNonContinuousModeCmd = K_STOP_CONTINUOUS_MODE;
    write(&setNonContinuousModeCmd, 1);

    // tell ahrs to save the configurations
    // char saveConfigurationsCmd = K_SAVE;
    // write(&saveConfigurationsCmd, 1);
  }


  void AhrsSerialInterface::read()
  {
    if (serialPtr_ == NULL){
      ROS_ERROR("read() called before init().  Calling init() now...");
      init();
    }

    // write getData command
    char getDataCmd = K_GET_DATA;
    write(&getDataCmd, 1);

    // read data from serial port
    std::string buffer;
    serialPtr_->readline(buffer);
/*
    // print the bytes of the buffer
    for(int ii = 0; ii < buffer.size(); ii++)
      ROS_INFO("packet[%d]: %02x", ii, (unsigned char)buffer.at(ii));
*/
    // check data packet
    if (
      check(buffer,
      calcCrc((unsigned char*) buffer.c_str(), buffer.size(), false)) )
    {
      parse(buffer);  // parse data packet
    }
  }


  void AhrsSerialInterface::write(char* commandCode, size_t length)
  {
    unsigned char command[4+length];
    command[0] = (4 + length) & 0xFF00;
    command[1] = (4 + length) & 0x00FF;

    for (int ii = 0; ii < length; ii++)
    {
      command[2 + ii] = commandCode[ii];
    }
    calcCrc(command, 4 + length, true);

    // write command to ahrs
    serialPtr_->write(command, 4 + length);
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
      memcpy(
        &yaw_,
        data[1].str().c_str(),
        sizeof(float));
      memcpy(
        &pitch_,
        data[2].str().c_str(),
        sizeof(float));
      memcpy(
        &roll_,
        data[3].str().c_str(),
        sizeof(float));
      for (int ii = 0; ii < 3; ii++)
      {
        memcpy(
          angularVelocity_ + ii,
          data[ii + 4].str().c_str(),
          sizeof(float));
        memcpy(
          linearAcceleration_ + ii,
          data[ii + 7].str().c_str(),
          sizeof(float));
      }

      ROS_INFO(" yaw: %f   pitch: %f   roll: %f", yaw_, pitch_, roll_);
/*
      ROS_INFO(
        "yaw[%f], pitch[%f], roll[%f], "
        "Ax[%f], Ay[%x], Az[%f], "
        "Gx[%f], Gy[%x], Gz[%f]",
        yaw_, pitch_, roll_,
        linearAcceleration_[0], linearAcceleration_[1], linearAcceleration_[0],
        angularVelocity_[0], angularVelocity_[1], angularVelocity_[2]);
*/
    }
    else
      ROS_ERROR("Did not match received packet to desirable pattern");
  }
}  // namespace imu
}  // namespace pandora_hardware_interface
