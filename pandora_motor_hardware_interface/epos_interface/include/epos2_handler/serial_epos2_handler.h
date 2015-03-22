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
* Author:     Konstantinos Panayiotou   <klpanagi@gmail.com>
* Maintainer: Konstantinos Panayiotou   <klpanagi@gmail.com>
*********************************************************************/
#ifndef EPOS_HANDLER_SERIAL_EPOS2_HANDLER_H
#define EPOS_HANDLER_SERIAL_EPOS2_HANDLER_H

#include "epos_handler/abstract_epos_handler.h"
#include "epos2_gateway/epos2_gateway.h"

namespace pandora_hardware_interface
{
namespace motor
{

  #define EPOS2_GATEWAY_ID          1
  #define RIGHT_FRONT_MOTOR_ID      1
  #define RIGHT_REAR_MOTOR_ID       2
  #define LEFT_FRONT_MOTOR_ID       3 
  #define LEFT_REAR_MOTOR_ID        4 
  #define RIGHT_FRONT_MOTOR_INDEX   0
  #define RIGHT_REAR_MOTOR_INDEX    1
  #define LEFT_FRONT_MOTOR_INDEX    2 
  #define LEFT_REAR_MOTOR_INDEX     3 
  #define NUM_NODES                 4
  
  struct Epos2Controller
  {
    uint16_t nodeId_;
    uint32_t errorCode_;
    std::string motorId_;
    uint16_t state_;
    int32_t rpm_;
    int16_t current_;
  };

  //=========================================================================

  class SerialEpos2Handler: public AbstractEposHandler
  {
    private:
      boost::scoped_ptr<Epos2Gateway> epos2Gateway_;
      std::vector<Epos2Controller*> epos2Controllers_;
      Epos2Controller* rightFrontMotor_;
      Epos2Controller* rightRearMotor_;
      Epos2Controller* leftFrontMotor_;
      Epos2Controller* leftRearMotor_;
    public:
      SerialEpos2Handler(const std::string port, 
        const uint32_t baudrate, const uint32_t timeout);
      virtual ~SerialEpos2Handler();
      virtual void getRPM(int* leftRearRpm, int* leftFrontRpm,
        int* rightRearRpm, int* rightFrontRpm);
      virtual void getCurrent(int* leftRearCurrent, int* leftFrontCurrent,
        int* rightRearCurrent, int* rightFrontCurrent);
      virtual Error getError();
      virtual uint16_t writeRPM(const int leftRpm, const int rightRpm);
      void readStates(void);
      void stateHandle(void);
     
  };
}  // namespace motor
}  // namespace pandora_hardware_interface

#endif  // EPOS_HANDLER_SERIAL_EPOS2_HANDLER_H
