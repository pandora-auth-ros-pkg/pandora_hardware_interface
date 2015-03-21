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

#include "epos2_handler/serial_epos2_handler.h"

namespace pandora_hardware_interface
{
namespace motor
{

  SerialEpos2Handler::SerialEpos2Handler( const std::string port, const uint32_t baudrate, const uint32_t timeout)
  {
    epos2Gateway.reset(new Epos2Gateway(port, baudrate, timeout, 
        NUM_NODES));
    nodeState_ = new uint16_t[NUM_NODES];
    epos2Gateway->openDevice();
    //resetDevice(1);
    //setEnableState(1);
    //epos2Gateway->resetNode(RIGHT_REAR_MOTOR);
    //epos2Gateway->resetNode(LEFT_FRONT_MOTOR);
    //epos2Gateway->resetNode(LEFT_REAR_MOTOR);
    //epos2Gateway->clearFaultState(1);
    //for (uint16_t i=1;i<NUM_NODES+1;i++)
    //{
      //epos2Gateway->setEnableState(i);
    //}
    stateHandle();
    stateHandle();
    epos2Gateway->activate_profileVelocityMode(0);
  }

  SerialEpos2Handler::~SerialEpos2Handler()
  {
    epos2Gateway->closeDevice();
  }

  void SerialEpos2Handler::stateHandle(void)
  {
    readStates();
    for(uint16_t _ii=0;_ii<NUM_NODES;_ii++)
    {
      switch(nodeState_[_ii])
      {
        case 0://DISABLE STATE
          epos2Gateway->resetNode(_ii+1);
          epos2Gateway->setEnableState(_ii+1);
          break;
        case 1://ENABLE STATE
          break;
        case 2://QUICKSTOP STATE
          epos2Gateway->setEnableState(_ii+1);
          break;
        case 3://FAULTY STATE
          epos2Gateway->clearFaultState(_ii+1);
          break;
        default:
          ROS_FATAL("[Motors]: UKNOWN STATE-->[%d]",
            nodeState_[_ii]);
      }
    }
  }


  void SerialEpos2Handler::getRPM(int* leftRearRpm, int* leftFrontRpm,
    int* rightRearRpm, int* rightFrontRpm)
  {
    //int32_t* nodeRpm = new int32_t[4];
    //read_velocityActual(0, &nodeRpm[0]);
    epos2Gateway->read_velocityActual(RIGHT_FRONT_MOTOR, rightFrontRpm);  
    epos2Gateway->read_velocityActual(RIGHT_REAR_MOTOR, rightRearRpm);  
    epos2Gateway->read_velocityActual(LEFT_FRONT_MOTOR, leftFrontRpm);  
    epos2Gateway->read_velocityActual(LEFT_REAR_MOTOR, leftRearRpm);  
  }

  void SerialEpos2Handler::getCurrent(int* leftRearCurrent, int* leftFrontCurrent,
    int* rightRearCurrent, int* rightFrontCurrent)
  {
    int16_t* nodeCurrent = new int16_t[4];
    //read_currentActual(0, &nodeCurrent[0]);
    epos2Gateway->read_currentActual(RIGHT_FRONT_MOTOR, 
      &nodeCurrent[RIGHT_FRONT_MOTOR]);
    epos2Gateway->read_currentActual(RIGHT_REAR_MOTOR, 
      &nodeCurrent[RIGHT_REAR_MOTOR]);
    epos2Gateway->read_currentActual(LEFT_FRONT_MOTOR, 
      &nodeCurrent[LEFT_FRONT_MOTOR]);
    epos2Gateway->read_currentActual(LEFT_REAR_MOTOR, 
      &nodeCurrent[LEFT_REAR_MOTOR]);
    *rightFrontCurrent = static_cast<int>(nodeCurrent[RIGHT_FRONT_MOTOR]);
    *rightRearCurrent = static_cast<int>(nodeCurrent[RIGHT_REAR_MOTOR]);
    *leftFrontCurrent = static_cast<int>(nodeCurrent[LEFT_FRONT_MOTOR]);
    *leftRearCurrent = static_cast<int>(nodeCurrent[LEFT_REAR_MOTOR]);
  }


  Error SerialEpos2Handler::getError() {}

  uint16_t SerialEpos2Handler::writeRPM( const int leftRpm, const int rightRpm) 
  {
    ROS_DEBUG("[Motors]: Setting speed %d, %d", leftRpm, rightRpm);
    epos2Gateway->moveWithVelocity(RIGHT_FRONT_MOTOR, rightRpm);
    epos2Gateway->moveWithVelocity(RIGHT_REAR_MOTOR, rightRpm);
    epos2Gateway->moveWithVelocity(LEFT_FRONT_MOTOR, leftRpm);
    epos2Gateway->moveWithVelocity(LEFT_REAR_MOTOR, leftRpm);
    return 1; //
  }

  void SerialEpos2Handler::readStates(void)
  {
    epos2Gateway->readState(RIGHT_FRONT_MOTOR,
     &nodeState_[RIGHT_FRONT_MOTOR]);
    epos2Gateway->readState(RIGHT_REAR_MOTOR,
     &nodeState_[RIGHT_FRONT_MOTOR]);
    epos2Gateway->readState(LEFT_FRONT_MOTOR,
     &nodeState_[RIGHT_FRONT_MOTOR]);
    epos2Gateway->readState(LEFT_REAR_MOTOR,
     &nodeState_[RIGHT_FRONT_MOTOR]);
  }


}  // namespace motor
}  // namespace pandora_hardware_interface

