/***************************************************************************
*   Copyright (C) 2010-2011 by Charalampos Serenis <info@devel.serenis.gr>*
*   Author: Charalampos Serenis <info@devel.serenis.gr>                   *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,  *
*   MA 02110-1301, USA.                                                   *
***************************************************************************/
#include "epos_handler/serial_epos2_handler.h"

namespace pandora_hardware_interface
{
namespace motor
{

  SerialEpos2Handler::SerialEpos2Handler( const std::string port, const uint32_t baudrate, const uint32_t timeout)
  {
    com_.deviceName = new char[32];
    com_.protocolStackName = new char[32];
    com_.interfaceName = new char[32];
    com_.portName = new char[32];
    com_.deviceName = "EPOS2";
    com_.protocolStackName = "MAXON_RS232";
    com_.interfaceName = "RS232";
    strcpy( com_.portName, port.c_str());
    com_.baudrate = baudrate;
    com_.timeout = timeout;
    nodeState_ = new uint16_t[4];
    openDevice();
    //resetDevice(1);
    //setEnableState(1);
    resetDevice(2);
    resetDevice(3);
    resetDevice(4);
    clearFault(1);
    for (uint16_t i=1;i<5;i++)
    {
      setEnableState(i);
    }
  }

  SerialEpos2Handler::~SerialEpos2Handler()
  {
    closeDevice();
    delete comHandler_;
    delete errorCode_;
  }

  void SerialEpos2Handler::openDevice()
  {
    /*--<Open Device Serial Communication interface>--*/
    comHandler_ = VCS_OpenDevice("EPOS2", "MAXON_RS232",
      "RS232", "/dev/ttyS0", errorCode_);
    if ( comHandler_==0 || *errorCode_!=0)
    {
      ROS_FATAL("[Motors]: Error while opening serial communication port");
      exit(-1);
    }
    /*--<Set and evaluate communication interface>--*/
    if( eval_communicationInterface()==0)
    {
      ROS_FATAL("[Motors]: Failed to evaluate communication parameters");
      exit(-1);
    }
    
  }

  void SerialEpos2Handler::closeDevice()
  {
    if(VCS_CloseDevice(comHandler_, errorCode_)!=0 && *errorCode_ == 0)
    {
      ROS_FATAL("[Motors]: Device communication port closed succesfully");
    }
    else
    {
      ROS_FATAL("[Motors]: Error closing device communication port...Investigate!!!"); 
    }

  }

  void SerialEpos2Handler::resetDevice(uint16_t nodeId)
  {
    if(VCS_ResetDevice(comHandler_, nodeId, errorCode_)!=0)
    {
      ROS_INFO("[Motors]: Reset NodeId[%d]", nodeId);
    }
    else
    {
      //TODO --- resolve error
    }
    
  }

  void SerialEpos2Handler::getRPM(int* leftRearRpm, int* leftFrontRpm,
    int* rightRearRpm, int* rightFrontRpm)
  {
    //int32_t* nodeRpm = new int32_t[4];
    //read_velocityActual(0, &nodeRpm[0]);
    read_velocityActual(RIGHT_FRONT_MOTOR, rightFrontRpm);  
    read_velocityActual(RIGHT_REAR_MOTOR, rightRearRpm);  
    read_velocityActual(LEFT_FRONT_MOTOR, leftFrontRpm);  
    read_velocityActual(LEFT_REAR_MOTOR, leftRearRpm);  
  }
  void SerialEpos2Handler::getCurrent( int* leftRearCurrent, int* leftFrontCurrent,
    int* rightRearCurrent, int* rightFrontCurrent)
  {
    int16_t* nodeCurrent = new int16_t[4];
    //read_currentActual(0, &nodeCurrent[0]);
    read_currentActual(RIGHT_FRONT_MOTOR, &nodeCurrent[RIGHT_FRONT_MOTOR]);
    read_currentActual(RIGHT_REAR_MOTOR, &nodeCurrent[RIGHT_REAR_MOTOR]);
    read_currentActual(LEFT_FRONT_MOTOR, &nodeCurrent[LEFT_FRONT_MOTOR]);
    read_currentActual(LEFT_REAR_MOTOR, &nodeCurrent[LEFT_REAR_MOTOR]);
    *rightFrontCurrent = static_cast<int>(nodeCurrent[RIGHT_FRONT_MOTOR]);
    *rightRearCurrent = static_cast<int>(nodeCurrent[RIGHT_REAR_MOTOR]);
    *leftFrontCurrent = static_cast<int>(nodeCurrent[LEFT_FRONT_MOTOR]);
    *leftRearCurrent = static_cast<int>(nodeCurrent[LEFT_REAR_MOTOR]);
  }

  bool SerialEpos2Handler::eval_communicationInterface()
  {
    uint32_t _baudrate;
    uint32_t _timeout;
    if(VCS_GetProtocolStackSettings(comHandler_, &_baudrate, 
        &_timeout, errorCode_)!=0)
    {
      if(VCS_SetProtocolStackSettings(comHandler_, com_.baudrate, 
          com_.timeout, errorCode_)!=0)
      {
        if(VCS_GetProtocolStackSettings(comHandler_, &_baudrate, 
            &_timeout, errorCode_)!=0)
	{
          if( com_.baudrate == _baudrate)
          {
            return true;
          }
        }
      }
    }
    return false;
  }

  void SerialEpos2Handler::setEnableState(uint16_t nodeId)
  {
    if(VCS_SetEnableState(comHandler_, nodeId, errorCode_)!=0)
    {
      ROS_INFO("[Motors]: NodeId[%d] is set at Enable State", nodeId);
    }
    else
    {
     //TODO --- Resolve error setting enable state
    }
  } 

  bool SerialEpos2Handler::isEnableState(uint16_t nodeId)
  {
    int isEnabled = 0;
    //isEnabled => 1: Device Enabled, 0: Device NOT Enabled
    if(VCS_GetEnableState(comHandler_, nodeId, &isEnabled, errorCode_)!=0)
    {
      if( isEnabled)
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

  void SerialEpos2Handler::setDisableState(uint16_t nodeId)
  {
    if(VCS_SetDisableState(comHandler_, nodeId, errorCode_)!=0)
    {
      ROS_INFO("[Motors]: NodeId[%d] is set at Disable State", nodeId);
    }
    else
    {
     //TODO --- Resolve error setting enable state
    }
  } 

  bool SerialEpos2Handler::isDisableState(uint16_t nodeId)
  {
    int isDisabled = 0;
    if(VCS_GetDisableState(comHandler_, nodeId, &isDisabled, errorCode_)!=0)
    {
      if( isDisabled)
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

  void SerialEpos2Handler::clearFault(uint16_t nodeId)
  {
    if(VCS_ClearFault(comHandler_, nodeId, errorCode_)!=0)
    {
      ROS_INFO("[Motors]: Cleared Fault State for NodeId[%d]", nodeId);
    }
    else
    {
      //TODO -- resolve errorCode
    }
  }

  bool SerialEpos2Handler::isFaultState(uint16_t nodeId)
  {
    int isFault = 0;
    if(VCS_GetFaultState(comHandler_, nodeId, &isFault, errorCode_)!=0)
    {
      if( isFault)
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

  bool SerialEpos2Handler::isQuickStopState(uint16_t nodeId)
  {
    int isQuickStopped = 0;
    if(VCS_GetQuickStopState(comHandler_, nodeId, &isQuickStopped, 
        errorCode_)!=0)
    {
      if( isQuickStopped)
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

  uint16_t SerialEpos2Handler::getState(uint16_t nodeId)
  {
    uint16_t _state = NULL;
    if(VCS_GetState(comHandler_, nodeId, &_state, errorCode_))
    {
      return _state;
    } 
    else
    {
      //TODO --- resolve errorCode
    }
  }


  Error SerialEpos2Handler::getError() {}
  uint16_t SerialEpos2Handler::writeRPM( const int leftRpm, const int rightRpm) 
  {
    ROS_DEBUG("[Motors]: Setting speed %d, %d", leftRpm, rightRpm);
    if(leftRpm==rightRpm)
    {
      moveWithVelocity(0, leftRpm);
    }
    else
    {
      moveWithVelocity(1, rightRpm);
      moveWithVelocity(2, rightRpm);
      moveWithVelocity(3, leftRpm);
      moveWithVelocity(4, leftRpm);
    }
    return 1; //
  }

  void SerialEpos2Handler::printState_all(void)
  {
    for (uint16_t i=1;i<5;i++)
    {
      switch(nodeState_[i])
      {
        case 0:
          ROS_INFO("[Motors]: NodeId[%d] is at Disabled state",i);
          break;
        case 1:
          ROS_INFO("[Motors]: NodeId[%d] is at Enabled state",i);
          break;
        case 2:
          ROS_INFO("[Motors]: NodeId[%d] is at QuickStop State",i);
          break;
        case 3:
          ROS_INFO("[Motors]: NodeId[%d] is at Faulty State",i);
          break;
        default:
          ROS_INFO("[Motors]: NodeId[%d], is at Unknown State",i);
          break;
      }
    }
  }
  
  void SerialEpos2Handler::getState_all(void)
  {
    for (uint16_t i=1;i<5;i++)
    {
      nodeState_[i-1] = getState(i);
    }
  }

  void SerialEpos2Handler::activate_profileVelocityMode(uint16_t nodeId)
  {
    if( nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<5;_ii++)
      {
        if(VCS_ActivateProfileVelocityMode(comHandler_, _ii, errorCode_)!=0)
        {
          ROS_INFO("[Motors]: NodeId[%d] is set at ProfileVelocity Mode", 
            _ii);
        }
        else
        {
          //TODO --- resolve errorCode
        }
      }
    }
    else
    {
      if(VCS_ActivateProfileVelocityMode(comHandler_, nodeId, errorCode_)!=0)
      {
        ROS_INFO("[Motors]: NodeId[%d] is set at ProfileVelocity Mode", 
          nodeId);
      }
      else
      {
        //TODO --- resolve errorCode
      }
    }
  }

  void SerialEpos2Handler::moveWithVelocity(uint16_t nodeId, int vel)
  {
    if( nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<5;_ii++)
      {
        if(VCS_MoveWithVelocity(comHandler_, _ii, vel, errorCode_)!=0)
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
      if(VCS_MoveWithVelocity(comHandler_, nodeId, vel, errorCode_)!=0)
      {
        //Nothing?!!!!
      } 
      else
      {
        //TODO --- resolve error
      }
    }
  }

  void SerialEpos2Handler::read_velocityActual(uint16_t nodeId, int32_t* velActual)
  {
    if( nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<5;_ii++)
      {
        if(VCS_GetVelocityIs(comHandler_, _ii, &velActual[_ii-1], errorCode_)!=0)
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
      if(VCS_GetVelocityIs(comHandler_, nodeId, velActual, errorCode_)!=0)
      {
        //Nothing?!!!!
      } 
      else
      {
        //TODO --- resolve error
      }
    }
  }

  void SerialEpos2Handler::read_velocityAvg(uint16_t nodeId, int32_t* velAvg)
  {
    if( nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<5;_ii++)
      {
        if(VCS_GetVelocityIsAveraged(comHandler_, _ii, &velAvg[_ii-1], errorCode_)!=0)
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
      if(VCS_GetVelocityIsAveraged(comHandler_, nodeId, velAvg, errorCode_)!=0)
      {
        //Nothing?!!!!
      } 
      else
      {
        //TODO --- resolve error
      }
    }
  }
  
  void SerialEpos2Handler::read_currentActual(uint16_t nodeId, int16_t* currentActual)
  {
    if( nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<5;_ii++)
      {
        if(VCS_GetCurrentIs(comHandler_, _ii, &currentActual[_ii-1], errorCode_)!=0)
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
      if(VCS_GetCurrentIs(comHandler_, nodeId, currentActual, errorCode_)!=0)
      {
        //Nothing?!!!!
      } 
      else
      {
        //TODO --- resolve error
      }
    }
  }
  
  void  SerialEpos2Handler::read_currentAvg(uint16_t nodeId, int16_t* currentAvg)
  {
    if( nodeId==0)
    {
      uint16_t _ii;
      for(_ii=1;_ii<5;_ii++)
      {
        if(VCS_GetCurrentIsAveraged(comHandler_, _ii, &currentAvg[_ii-1], errorCode_)!=0)
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
      if(VCS_GetCurrentIsAveraged(comHandler_, nodeId, currentAvg, errorCode_)!=0)
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

