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
#include "epos2_gateway/epos2_gateway.h"


namespace pandora_hardware_interface
{
namespace motor
{
  Epos2Gateway::Epos2Gateway(const std::string port, const uint32_t baudrate, const uint32_t timeout, 
    const uint16_t numOfNodes)
  {
    comInterface_.reset(new Interface());
    comInterface_->deviceName = new char[32];
    comInterface_->protocolStackName = new char[32];
    comInterface_->interfaceName = new char[32];
    comInterface_->portName = new char[32];
    strcpy(comInterface_->deviceName, "EPOS2");
    strcpy(comInterface_->protocolStackName, "MAXON_RS232");
    strcpy(comInterface_->interfaceName, "RS232");
    strcpy(comInterface_->portName, port.c_str());
    comInterface_->baudrate = baudrate;
    comInterface_->timeout = timeout;
    //TODO --- Read number of Nodes from a yaml
    numOfNodes_ = numOfNodes;
    nodeError_ = new uint32_t[numOfNodes_];
  }

  Epos2Gateway::~Epos2Gateway()
  {
    //delete comHandler_; 
  }

  void Epos2Gateway::openDevice()
  {
    /*--<Open Device Serial Communication interface>--*/
    comHandler_ = VCS_OpenDevice(comInterface_->deviceName,
      comInterface_->protocolStackName, comInterface_->interfaceName, 
      comInterface_->portName, &comInterface_->error);
    if (comHandler_==0 || comInterface_->error!=0)
    {
      ROS_FATAL("[Motors]: Error while opening serial\
        communication port");
      exit(-1);
    }
    else
    {
      ROS_FATAL("[Motors]: Opened communicaiton port, %s",
        comInterface_->portName);
    }
    /*--<Set and evaluate communication interface>--*/
    if(eval_communicationInterface()==0)
    {
      ROS_FATAL("[Motors]: Failed to evaluate communication\
        parameters");
      exit(-1);
    }

  }

  void Epos2Gateway::closeDevice()
  {
    if(VCS_CloseDevice(comHandler_, &comInterface_->error)!=0 &&
      comInterface_->error == 0)
    {
      ROS_FATAL("[Motors]: Device communication port\
        closed succesfully");
    }
    else
    {
      ROS_FATAL("[Motors]: Error closing device\
        communication port...Investigate!!!"); 
    }

  }

  void Epos2Gateway::resetNode(uint16_t nodeId)
  {
    if(nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<numOfNodes_+1;_ii++)
      {
        if(VCS_ResetDevice(comHandler_, _ii, &nodeError_[_ii-1])!=0)
        {
          ROS_INFO("[Motors]: Reset NodeId[%d]", _ii);
        }
        else
        {
          //TODO --- resolve errorCode
        }
      }
    }
    else
    {
      if(VCS_ResetDevice(comHandler_, nodeId, &nodeError_[nodeId-1])!=0)
      {
        ROS_INFO("[Motors]: Reset NodeId[%d]", nodeId);
      }
      else
      {
        //TODO --- resolve errorCode
      }
    }
  }

  bool Epos2Gateway::eval_communicationInterface()
  {
    uint32_t _baudrate;
    uint32_t _timeout;
    if(VCS_GetProtocolStackSettings(comHandler_, &_baudrate, 
        &_timeout, &comInterface_->error)!=0)
    {
      if(VCS_SetProtocolStackSettings(comHandler_, comInterface_->baudrate, 
          comInterface_->timeout, &comInterface_->error)!=0)
      {
        if(VCS_GetProtocolStackSettings(comHandler_, &_baudrate, 
            &_timeout, &comInterface_->error)!=0)
        {
          if(comInterface_->baudrate == _baudrate)
          {
            return true;
          }
        }
      }
    }
    return false;
  }

 
  void Epos2Gateway::setEnableState(uint16_t nodeId)
  {
    if(nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<numOfNodes_+1;_ii++)
      {
        if(VCS_SetEnableState(comHandler_, _ii, &nodeError_[_ii-1])!=0)
        {
          ROS_INFO("[Motors]: NodeId[%d] is set at {Enabled-State}", _ii);
        }
        else
        {
          //TODO --- resolve errorCode
        }
      }
    }
    else
    {
      if(VCS_SetEnableState(comHandler_, nodeId, &nodeError_[nodeId-1])!=0)
      {
        ROS_INFO("[Motors]: NodeId[%d] is set at {Enabled-State}", nodeId);
      }
      else
      {
        //TODO --- resolve errorCode
      }
    }
  } 


  bool Epos2Gateway::isEnableState(uint16_t nodeId)
  {
    int isEnabled = 0;
    //isEnabled => 1: Device Enabled, 0: Device NOT Enabled
    if(VCS_GetEnableState(comHandler_, nodeId, &isEnabled, &nodeError_[nodeId-1])!=0)
    {
      if(isEnabled)
      {
        //Device is at Enabled State
        return true;
      } 
      else
      {
        //Device is NOT at Enabled State
        return false;
      }
    }
    else
    {
      //TODO --- Resolve errorCode
    }
  }


  void Epos2Gateway::setDisableState(uint16_t nodeId)
  {
    if(nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<numOfNodes_+1;_ii++)
      {
        if(VCS_SetDisableState(comHandler_, _ii, &nodeError_[_ii-1])!=0)
        {
          ROS_INFO("[Motors]: NodeId[%d] is set at Disabled state", _ii);
        }
        else
        {
          //TODO --- resolve errorCode
        }
      }
    }
    else
    {
      if(VCS_SetDisableState(comHandler_, nodeId, &nodeError_[nodeId-1])!=0)
      {
        ROS_INFO("[Motors]: NodeId[%d] is set at Disabled State", nodeId);
      }
      else
      {
        //TODO --- resolve errorCode
      }
    }
  } 


  bool Epos2Gateway::isDisableState(uint16_t nodeId)
  {
    int isDisabled = 0;
    if(VCS_GetDisableState(comHandler_, nodeId, &isDisabled,
        &nodeError_[nodeId-1])!=0)
    {
      if(isDisabled)
      {
        //Device is at Disabled State
        return true;
      } 
      else
      {
        //Device is NOT at Disabled State
        return false;
      }
    }
    else
    {
      //TODO --- Resolve errorCode 
    }
  }

  void Epos2Gateway::clearFaultState(uint16_t nodeId)
  {
    if(nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<numOfNodes_+1;_ii++)
      {
        if(VCS_ClearFault(comHandler_, _ii, &nodeError_[_ii-1])!=0)
        {
          ROS_INFO("[Motors]: Cleared Fault state of NodeId[%d]", _ii);
        }
        else
        {
          //TODO --- resolve errorCode
        }
      }
    }
    else
    {
      if(VCS_ClearFault(comHandler_, nodeId, &nodeError_[nodeId-1])!=0)
      {
        ROS_INFO("[Motors]: Cleared Fault state of NodeId[%d]", nodeId);
      }
      else
      {
        //TODO --- resolve errorCode
      }
    }
  }


  bool Epos2Gateway::isFaultState(uint16_t nodeId)
  {
    int isFault = 0;
    if(VCS_GetFaultState(comHandler_, nodeId, &isFault, &nodeError_[nodeId-1])!=0)
    {
      if(isFault)
      {
        //Device is at Fault State
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      //TODO --- resolve errorCode
    }
  }


  bool Epos2Gateway::isQuickStopState(uint16_t nodeId)
  {
    int isQuickStopped = 0;
    if(VCS_GetQuickStopState(comHandler_, nodeId, &isQuickStopped, 
        &nodeError_[nodeId-1])!=0)
    {
      if(isQuickStopped)
      {
        //Device is at "QuickStop" State
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      //TODO --- resolve errorCode
    }
  }


  void Epos2Gateway::setQuickStopState(uint16_t nodeId)
  {
    if(nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<numOfNodes_+1;_ii++)
      {
        if(VCS_SetQuickStopState(comHandler_, _ii, &nodeError_[_ii-1])!=0)
        {
          ROS_INFO("[Motors]: NodeId[%d] is set at QuickStop state", _ii);
        }
        else
        {
          //TODO --- resolve errorCode
        }
      }
    }
    else
    {
      if(VCS_SetQuickStopState(comHandler_, nodeId, &nodeError_[nodeId-1])!=0)
      {
        ROS_INFO("[Motors]: NodeId[%d] is set at QuickStop state", nodeId);
      }
      else
      {
        //TODO --- resolve errorCode
      }
    }
  }


  void Epos2Gateway::readState(uint16_t nodeId, uint16_t* nodeState)
  {
    if(VCS_GetState(comHandler_, nodeId, nodeState,
        &nodeError_[nodeId-1])!=0)
    {
      //TODO --- Do what?
    }
    else
    {
      //TODO --- resolve errorCode
      ROS_FATAL("[Motors]: Error while trying to read NodeId[%d]  State",
        nodeId);
      *nodeState = 9;
    }
  }


  void Epos2Gateway::activate_profileVelocityMode(uint16_t nodeId)
  {
    if(nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<numOfNodes_+1;_ii++)
      {
        if(VCS_ActivateProfileVelocityMode(comHandler_, _ii,
            &nodeError_[_ii-1])!=0)
        {
          ROS_INFO("[Motors]: NodeId[%d] is set at ProfileVelocity Mode", _ii);
        }
        else
        {
          //TODO --- resolve errorCode
        }
      }
    }
    else
    {
      if(VCS_ActivateProfileVelocityMode(comHandler_, nodeId,
          &nodeError_[nodeId-1])!=0)
      {
        ROS_INFO("[Motors]: NodeId[%d] is set at ProfileVelocity Mode", nodeId);
      }
      else
      {
        //TODO --- resolve errorCode
      }
    }
  }


  void Epos2Gateway::moveWithVelocity(uint16_t nodeId, int vel)
  {
    if(nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<numOfNodes_+1;_ii++)
      {
        if(VCS_MoveWithVelocity(comHandler_, _ii, vel, &nodeError_[_ii-1])!=0)
        {
          //Nothing?!!!!
        } 
        else
        {
          //TODO --- resolve error
        }
      }
    }
    else
    {
      if(VCS_MoveWithVelocity(comHandler_, nodeId, vel,
          &nodeError_[nodeId-1])!=0)
      {
        //Nothing?!!!!
      } 
      else
      {
        //TODO --- resolve error
      }
    }
  }

  void Epos2Gateway::read_velocityActual(uint16_t nodeId, int32_t* velActual)
  {
    if(nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<numOfNodes_+1;_ii++)
      {
        if(VCS_GetVelocityIs(comHandler_, _ii, &velActual[_ii-1],
            &nodeError_[_ii-1])!=0)
        {
          //Nothing?!!!!
        } 
        else
        {
          //TODO --- resolve error
        }
      }
    }
    else
    {
      if(VCS_GetVelocityIs(comHandler_, nodeId, velActual,
          &nodeError_[nodeId-1])!=0)
      {
        //Nothing?!!!!
      } 
      else
      {
        //TODO --- resolve error
      }
    }
  }

  void Epos2Gateway::read_velocityAvg(uint16_t nodeId, int32_t* velAvg)
  {
    if(nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<numOfNodes_+1;_ii++)
      {
        if(VCS_GetVelocityIsAveraged(comHandler_, _ii, &velAvg[_ii-1],
            &nodeError_[_ii-1])!=0)
        {
          //Nothing?!!!!
        } 
        else
        {
          //TODO --- resolve error
        }
      }
    }
    else
    {
      if(VCS_GetVelocityIsAveraged(comHandler_, nodeId, velAvg,
          &nodeError_[nodeId-1])!=0)
      {
        //TODO --- Do Nothing?!!!!
      } 
      else
      {
        //TODO --- resolve error
      }
    }
  }
  

  void Epos2Gateway::read_currentActual(uint16_t nodeId, int16_t* currentActual)
  {
    if(nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<numOfNodes_+1;_ii++)
      {
        if(VCS_GetCurrentIs(comHandler_, _ii, &currentActual[_ii-1],
            &nodeError_[_ii-1])!=0)
        {
          //TODO --- Do Nothing?!!!!
        } 
        else
        {
          //TODO --- resolve error
        }
      }
    }
    else
    {
      if(VCS_GetCurrentIs(comHandler_, nodeId, currentActual,
          &nodeError_[nodeId-1])!=0)
      {
        //Nothing?!!!!
      } 
      else
      {
        //TODO --- resolve error
      }
    }
  }
  

  void Epos2Gateway::read_currentAvg(uint16_t nodeId, int16_t* currentAvg)
  {
    if(nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<numOfNodes_+1;_ii++)
      {
        if(VCS_GetCurrentIsAveraged(comHandler_, _ii, &currentAvg[_ii-1],
            &nodeError_[_ii-1])!=0)
        {
          //TODO --- Do Nothing?!!!!
        }
        else
        {
          //TODO --- resolve error
        }
      }
    }
    else
    {
      if(VCS_GetCurrentIsAveraged(comHandler_, nodeId, currentAvg,
          &nodeError_[nodeId-1])!=0)
      {
        //Nothing?!!!!
      } 
      else
      {
        //TODO --- resolve error
      }
    }
  }

}  // namespace motor
}  // namespace pandora_hardware_interface

