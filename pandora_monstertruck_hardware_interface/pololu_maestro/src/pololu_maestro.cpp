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

#include "pololu_maestro/pololu_maestro.h"

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

  bool PololuMaestro::setTarget(uint8_t channel, double target)
  {
    // convert target from radians to degrees
    uint16_t angle = static_cast<uint16_t>(
      std::max(std::min(180.0 - target * 180.0 / M_PI, 180.0), 0.0));

    // convert angle from degrees to quarter microseconds
    uint16_t command = static_cast<uint16_t>(angle * 10.48 + 496)*4;

    uint8_t cmdBuffer[] =
      {0x84, channel, command & 0x7F, (command >> 7) & 0x7F};

    // flush input and output buffers
    serialPtr_->flush();

    // write command
    int sentBytes =
      serialPtr_->write(
        cmdBuffer,
        sizeof(cmdBuffer)/sizeof(uint8_t));

    if (sentBytes == sizeof(cmdBuffer)/sizeof(uint8_t))
      return true;
    else
      return false;
  }

  double PololuMaestro::readPosition(uint8_t channel)
  {
    // request feedback for the given channel
    uint8_t cmdBuffer[] = {0x90, channel};
    serialPtr_->write(cmdBuffer, sizeof(cmdBuffer)/sizeof(uint8_t));

    // receive feedback
    uint8_t response[2];
    serialPtr_->read(response, sizeof(response)/sizeof(uint8_t));
    return
      M_PI / 180.0 * (180.0 -
        (static_cast<double>(256*response[1] + response[0]) / 4 - 496) / 10.48);
  }

  double PololuMaestro::readVoltage(uint8_t channel)
  {
    // request feedback for the given channel
    uint8_t cmdBuffer[] = {0x90, channel};
    serialPtr_->write(cmdBuffer, sizeof(cmdBuffer)/sizeof(uint8_t));

    // receive feedback
    uint8_t response[2];
    serialPtr_->read(response, sizeof(response)/sizeof(uint8_t));

    return (static_cast<double>(256*response[1] + response[0]) / 1024 * 5);
  }

  void PololuMaestro::readErrors()
  {
    // request to read error
    uint8_t cmdBuffer[] = {0xa1};
    serialPtr_->write(cmdBuffer, sizeof(cmdBuffer)/sizeof(uint8_t));

    // read response
    uint8_t response[2];
    serialPtr_->read(response, sizeof(response)/sizeof(uint8_t));

    if (response[0] | response[1])
      ROS_WARN("[pololu maestro] Error: 0x%x%x", response[1], response[0]);
  }

}  // namespace pololu_maestro
}  // namespace pandora_hardware_interface
