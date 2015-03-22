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

  SerialEpos2Handler::SerialEpos2Handler(const std::string port, const uint32_t baudrate, const uint32_t timeout)
  {
    epos2Gateway_.reset(new Epos2Gateway(port, baudrate, timeout, 
        NUM_NODES));
    rightFrontMotor_ = new Epos2Controller();
    epos2Controllers_.push_back(rightFrontMotor_);
    rightFrontMotor_->nodeId_ = RIGHT_FRONT_MOTOR_ID;
    rightRearMotor_ = new Epos2Controller();
    epos2Controllers_.push_back(rightRearMotor_);
    rightRearMotor_->nodeId_ = RIGHT_REAR_MOTOR_ID;
    leftFrontMotor_ = new Epos2Controller();
    epos2Controllers_.push_back(leftFrontMotor_);
    leftFrontMotor_->nodeId_ = LEFT_FRONT_MOTOR_ID;
    leftRearMotor_ = new Epos2Controller();
    epos2Controllers_.push_back(leftRearMotor_);
    leftRearMotor_->nodeId_ = LEFT_REAR_MOTOR_ID;

    rightFrontMotor_->state_ = 0;
    rightRearMotor_->state_ = 0;
    leftFrontMotor_->state_ = 0;
    leftRearMotor_->state_ = 0;
    epos2Gateway_->openDevice();
    /*--<Initialize motor controller states {Enabled}>---*/
    readStates();
    stateHandle();
    /*----------------------------------------------------*/
    //Set every epos2 controller at profileVelocityMode on startup 
    epos2Gateway_->activate_profileVelocityMode(0);
    //epos2Gateway_->activate_profileVelocityMode(RIGHT_FRONT_MOTOR_ID);
    //epos2Gateway_->activate_profileVelocityMode(RIGHT_REAR_MOTOR_ID);
    //epos2Gateway_->activate_profileVelocityMode(LEFT_FRONT_MOTOR_ID);
    //epos2Gateway_->activate_profileVelocityMode(LEFT_REAR_MOTOR_ID);
  }

  SerialEpos2Handler::~SerialEpos2Handler()
  {
    epos2Gateway_->moveWithVelocity(0,0);
    epos2Gateway_->closeDevice();
  }

  void SerialEpos2Handler::stateHandle(void)
  {
    for(uint16_t _ii=0;_ii<epos2Controllers_.size();_ii++)
    {
      switch(epos2Controllers_.at(_ii)->state_)
      {
        case 0://DISABLE STATE
          epos2Gateway_->resetNode(epos2Controllers_.at(_ii)->nodeId_);
          epos2Gateway_->setEnableState(epos2Controllers_.at(_ii)->nodeId_);
          break;
        case 1://ENABLE STATE
          break;
        case 2://QUICKSTOP STATE
          epos2Gateway_->setEnableState(epos2Controllers_.at(_ii)->nodeId_);
          break;
        case 3://FAULTY STATE
          epos2Gateway_->clearFaultState(epos2Controllers_.at(_ii)->nodeId_);
          epos2Gateway_->setEnableState(epos2Controllers_.at(_ii)->nodeId_);
          break;
        case 9://CANNOT COMMUNICATE
          if(epos2Controllers_.at(_ii)->nodeId_==EPOS2_GATEWAY_ID)
          {
            //Cannot communicate with epos2-Gateway
            ROS_FATAL("[Motors]: Cannot communicate with epos2-Gateway.");
          }
          epos2Gateway_->resetNode(epos2Controllers_.at(_ii)->nodeId_);
          epos2Gateway_->setEnableState(epos2Controllers_.at(_ii)->nodeId_);
          break;
        default://UKNOWN STATE
          ROS_FATAL("[Motors]: UKNOWN STATE --> [%d]",
            epos2Controllers_.at(_ii)->state_);
          epos2Gateway_->resetNode(epos2Controllers_.at(_ii)->nodeId_);
          epos2Gateway_->setEnableState(epos2Controllers_.at(_ii)->nodeId_);
          break;
      }
    }
    readStates();
    for(uint16_t _ii=0;_ii<epos2Controllers_.size();_ii++)
    {
      if(epos2Controllers_.at(_ii)->state_!=1)
      {
        stateHandle();
      }
    }
  }


  void SerialEpos2Handler::getRPM(int* leftRearRpm, int* leftFrontRpm,
    int* rightRearRpm, int* rightFrontRpm)
  {
    epos2Gateway_->read_velocityActual(rightFrontMotor_->nodeId_, 
      &rightFrontMotor_->rpm_);  
    epos2Gateway_->read_velocityActual(rightRearMotor_->nodeId_,
      &rightRearMotor_->rpm_);  
    epos2Gateway_->read_velocityActual(leftFrontMotor_->nodeId_,
      &leftFrontMotor_->rpm_);  
    epos2Gateway_->read_velocityActual(leftRearMotor_->nodeId_,
      &leftRearMotor_->rpm_);  
    *rightFrontRpm = rightFrontMotor_->rpm_;
    *rightRearRpm = rightRearMotor_->rpm_;
    *leftFrontRpm = leftFrontMotor_->rpm_;
    *leftRearRpm = leftRearMotor_->rpm_;
  }

  void SerialEpos2Handler::getCurrent(int* leftRearCurrent, int* leftFrontCurrent,
    int* rightRearCurrent, int* rightFrontCurrent)
  {
    epos2Gateway_->read_currentActual(rightFrontMotor_->nodeId_, 
      &rightFrontMotor_->current_);
    epos2Gateway_->read_currentActual(rightRearMotor_->nodeId_,
      &rightRearMotor_->current_);
    epos2Gateway_->read_currentActual(leftFrontMotor_->nodeId_, 
      &rightFrontMotor_->current_);
    epos2Gateway_->read_currentActual(leftRearMotor_->nodeId_, 
      &leftRearMotor_->current_);
    *rightFrontCurrent = static_cast<int>(rightFrontMotor_->current_);
    *rightRearCurrent = static_cast<int>(rightRearMotor_->current_);
    *leftFrontCurrent = static_cast<int>(leftFrontMotor_->current_);
    *leftRearCurrent = static_cast<int>(leftRearMotor_->current_);
  }


  Error SerialEpos2Handler::getError() {}

  uint16_t SerialEpos2Handler::writeRPM( const int leftRpm, const int rightRpm) 
  {
    ROS_DEBUG("[Motors]: Setting speed %d, %d", leftRpm, rightRpm);
    epos2Gateway_->moveWithVelocity(rightFrontMotor_->nodeId_, -rightRpm);
    epos2Gateway_->moveWithVelocity(rightRearMotor_->nodeId_, -rightRpm);
    epos2Gateway_->moveWithVelocity(leftFrontMotor_->nodeId_, leftRpm);
    epos2Gateway_->moveWithVelocity(leftRearMotor_->nodeId_, leftRpm);
    return 1; //
  }

  void SerialEpos2Handler::readStates(void)
  {
    epos2Gateway_->readState(rightFrontMotor_->nodeId_,
      &rightFrontMotor_->state_);
    ROS_INFO("[Motors]: NodeId-%d State==%d", rightFrontMotor_->nodeId_, 
      rightFrontMotor_->state_);
    epos2Gateway_->readState(rightRearMotor_->nodeId_,
      &rightRearMotor_->state_);
    ROS_INFO("[Motors]: NodeId-%d State==%d", rightRearMotor_->nodeId_, 
      rightRearMotor_->state_);
    epos2Gateway_->readState(leftFrontMotor_->nodeId_,
      &leftFrontMotor_->state_);
    ROS_INFO("[Motors]: NodeId-%d State==%d", leftFrontMotor_->nodeId_, 
      leftFrontMotor_->state_);
    epos2Gateway_->readState(leftRearMotor_->nodeId_,
     &leftRearMotor_->state_);
    ROS_INFO("[Motors]: NodeId-%d State==%d", leftRearMotor_->nodeId_, 
      leftRearMotor_->state_);
  }

  //=======================================================================
  
}  // namespace motor
}  // namespace pandora_hardware_interface

