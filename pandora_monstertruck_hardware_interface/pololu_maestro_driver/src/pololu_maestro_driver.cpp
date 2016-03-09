/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, P.A.N.D.O.R.A. Team.
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

#include "pololu_maestro_driver/pololu_maestro_driver.h"

namespace pandora_hardware_interface
{
namespace pololu_maestro
{
  PololuMaestro::PololuMaestro(
    const std::string& portName,
    int baudRate,
    int timeout)
  {
    try
    {
      serialPtr_.reset(new serial::Serial(
        portName,
        baudRate,
        serial::Timeout::simpleTimeout(timeout)));
    }
    catch (serial::IOException& ex)
    {
      ROS_FATAL("[Pololu Maestro] cannot open port[%s]", portName.c_str());
      ROS_FATAL("%s", ex.what());
      exit(-1);
    }

    readErrors();
  }

  PololuMaestro::~PololuMaestro()
  {
    serialPtr_->close();
  }

  bool PololuMaestro::setTarget(unsigned char channel, double target)
  {
    // convert target from radians to degrees
    unsigned short angle = static_cast<unsigned short>(
      std::max(std::min(target * 180.0 / M_PI, 180.0), 0.0));

    // convert angle from degrees to quarter microseconds
    unsigned short command = static_cast<unsigned short>(angle * 10.48 + 496)*4;

    unsigned char cmdBuffer[] =
      {0x84, 0x00, command & 0x7F, (command >> 7) & 0x7F};

    int sentBytes =
      serialPtr_->write(
        cmdBuffer,
        sizeof(cmdBuffer)/sizeof(unsigned char));

    if (sentBytes == sizeof(cmdBuffer)/sizeof(unsigned char))
      return true;
    else
      return false;
  }

  double PololuMaestro::readPosition(unsigned char channel)
  {
    // request feedback for the given channel
    unsigned char cmdBuffer[] = {0x90, channel};
    serialPtr_->write(cmdBuffer, sizeof(cmdBuffer)/sizeof(unsigned char));

    // receive feedback
    unsigned char response[2];
    serialPtr_->read(response, sizeof(response)/sizeof(unsigned char));
    return
      (static_cast<double>(256*response[1] + response[0]) / 4 - 496) / 10.48;
  }

  double PololuMaestro::readVoltage(unsigned char channel)
  {
    // request feedback for the given channel
    unsigned char cmdBuffer[] = {0x90, channel};
    serialPtr_->write(cmdBuffer, sizeof(cmdBuffer)/sizeof(unsigned char));

    // receive feedback
    unsigned char response[2];
    serialPtr_->read(response, sizeof(response)/sizeof(unsigned char));

    return (static_cast<double>(256*response[1] + response[0]) / 1024 * 5);
  }

  void PololuMaestro::readErrors()
  {
    // request to read error
    unsigned char cmdBuffer[] = {0xa1};
    serialPtr_->write(cmdBuffer, sizeof(cmdBuffer)/sizeof(unsigned char));

    // read response
    unsigned char response[2];
    serialPtr_->read(response, sizeof(response)/sizeof(unsigned char));

    ROS_INFO("[pololu maestro] Error Code: %x %x", response[1], response[0]);
  }

}  // namespace pololu_maestro
}  // namespace pandora_hardware_interface
