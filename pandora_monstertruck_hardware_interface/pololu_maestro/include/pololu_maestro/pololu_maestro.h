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

#ifndef POLOLU_MAESTRO_POLOLU_MAESTRO_H
#define POLOLU_MAESTRO_POLOLU_MAESTRO_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <boost/scoped_ptr.hpp>
#include <math.h>
#include <algorithm>

namespace pandora_hardware_interface
{
namespace pololu_maestro
{
  /*
   * @class PololuMaestro
   * @brief Class that controls pololu maestro servo controllers
   */
  class PololuMaestro
  {
   public:
    /*
     * @brief Constructor
     */
    PololuMaestro(const std::string& portName, int baudRate, int timeout);

    /*
     * @brief Destructor
     */
    ~PololuMaestro();

    /*
     * @brief Writes position commands to pololu maestro on the given channel
     * @param channel [unsigned char] : servo channel to command
     * @param target [double] : position command in radians
     * @return true : command was written successfully
     * @return false : command wasn't written successfully
     */
    bool setTarget(unsigned char channel, double target);

    /*
     * @brief Read the current position of the servo in the given channel
     * @param channel [unsigned char] : the channel to read feedback from
     * @return double : the position feedback on the given channel
     */
    double readPosition(unsigned char channel);

    /*
     * @brief Read the voltage on the given channel
     * @param channel [unsigned char] : the channel to read the voltage from
     * @return doubel : voltage measurement
     */
    double readVoltage(unsigned char channel);

    /*
     * @brief Read and clear errors (required due to startup serial error)
     * @return void
     */
    void readErrors();

   private:
    boost::scoped_ptr<serial::Serial> serialPtr_;  //!< serial controller pointer

  };  // class PololuMaestro

}  // namespace pololu_maestro
}  // namespace pandora_hardware_interface
#endif  // POLOLU_MAESTRO_POLOLU_MAESTRO_H
