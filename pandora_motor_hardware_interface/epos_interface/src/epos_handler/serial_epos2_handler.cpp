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

  SerialEpos2Handler::SerialEpos2Handler( const std::string port, const unsigned int baudrate, const unsigned int timeout )
  {
    com_.deviceName = new char[32];
    com_.protocolStackName = new char[32];
    com_.interfaceName = new char[32];
    com_.portName = new char[32];
    com_.deviceName = "EPOS2";
    com_.protocolStackName = "MAXON_RS232";
    com_.interfaceName = "RS232";
    strcpy( com_.portName, port.c_str() );
    com_.baudrate = baudrate;
    com_.timeout = timeout;
    nodeState_ = new unsigned short[4];
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
    if ( comHandler_==0 || *errorCode_!=0 )
    {
      ROS_FATAL("[Motors]: Error while opening serial communication port");
      exit(-1);
    }
    /*--<Set and evaluate communication interface>--*/
    if( eval_communicationInterface()==0 )
    {
      ROS_FATAL("[Motors]: Failed to evaluate communication parameters");
      exit(-1);
    }
    
  }

  void SerialEpos2Handler::closeDevice()
  {
    if( VCS_CloseDevice(comHandler_, errorCode_)!=0 && *errorCode_ == 0 )
    {
      ROS_FATAL("[Motors]: Device communication port closed succesfully");
    }
    else
    {
      ROS_FATAL("[Motors]: Error closing device communication port...Investigate!!!"); 
    }

  }

  void SerialEpos2Handler::resetDevice(unsigned short nodeId)
  {
    if( VCS_ResetDevice(comHandler_, nodeId, errorCode_)!=0 )
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
  
  }
  void SerialEpos2Handler::getCurrent( int* leftRearCurrent, int* leftFrontRpm,
    int* rightRearCurrent, int* rightFrontCurrent )
  {
  
  }

  bool SerialEpos2Handler::eval_communicationInterface()
  {
    unsigned int _baudrate;
    unsigned int _timeout;
    if( VCS_GetProtocolStackSettings(comHandler_, &_baudrate, 
        &_timeout, errorCode_ )!=0 )
    {
      if( VCS_SetProtocolStackSettings(comHandler_, com_.baudrate, 
          com_.timeout, errorCode_)!=0 )
      {
        if( VCS_GetProtocolStackSettings(comHandler_, &_baudrate, 
            &_timeout, errorCode_)!=0 )
	{
          if( com_.baudrate == _baudrate )
          {
            return true;
          }
        }
      }
    }
    return false;
  }

  void SerialEpos2Handler::setEnableState(unsigned short nodeId)
  {
    if( VCS_SetEnableState(comHandler_, nodeId, errorCode_)!=0 )
    {
      ROS_INFO("[Motors]: NodeId[%d] is set at Enable State", nodeId);
    }
    else
    {
     //TODO --- Resolve error setting enable state
    }
  } 

  bool SerialEpos2Handler::isEnableState(unsigned short nodeId)
  {
    int isEnabled = 0;
    //isEnabled => 1: Device Enabled, 0: Device NOT Enabled
    if( VCS_GetEnableState(comHandler_, nodeId, &isEnabled, errorCode_)!=0 )
    {
      if( isEnabled )
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

  void SerialEpos2Handler::setDisableState(unsigned short nodeId)
  {
    if( VCS_SetDisableState(comHandler_, nodeId, errorCode_)!=0 )
    {
      ROS_INFO("[Motors]: NodeId[%d] is set at Disable State", nodeId);
    }
    else
    {
     //TODO --- Resolve error setting enable state
    }
  } 

  bool SerialEpos2Handler::isDisableState(unsigned short nodeId)
  {
    int isDisabled = 0;
    if( VCS_GetDisableState(comHandler_, nodeId, &isDisabled, errorCode_)!=0 )
    {
      if( isDisabled )
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

  void SerialEpos2Handler::clearFault(unsigned short nodeId)
  {
    if( VCS_ClearFault(comHandler_, nodeId, errorCode_)!=0 )
    {
      ROS_INFO("[Motors]: Cleared Fault State for NodeId[%d]", nodeId);
    }
    else
    {
      //TODO -- resolve errorCode
    }
  }

  bool SerialEpos2Handler::isFaultState(unsigned short nodeId)
  {
    int isFault = 0;
    if( VCS_GetFaultState(comHandler_, nodeId, &isFault, errorCode_)!=0 )
    {
      if( isFault )
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

  bool SerialEpos2Handler::isQuickStopState(unsigned short nodeId)
  {
    int isQuickStopped = 0;
    if( VCS_GetQuickStopState(comHandler_, nodeId, &isQuickStopped, 
        errorCode_)!=0 )
    {
      if( isQuickStopped )
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

  unsigned short SerialEpos2Handler::getState(unsigned short nodeId)
  {
    unsigned short _state = NULL;
    if( VCS_GetState(comHandler_, nodeId, &_state, errorCode_) )
    {
      return _state;
    } 
    else
    {
      //TODO --- resolve errorCode
    }
  }


  Error SerialEpos2Handler::getError() {}
  unsigned short SerialEpos2Handler::writeRPM( const int leftRpm, const int rightRpm ) {}

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

  void SerialEpos2Handler::activate_profileVelocityMode(unsigned short nodeId)
  {
    if( nodeId==0 )
    {
      unsigned short _ii;
      for(_ii=1;_ii<5;_ii++)
      {
        if( VCS_ActivateProfileVelocityMode(comHandler_, _ii, errorCode_)!=0 )
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

  void SerialEpos2Handler::moveWithVelocity(unsigned short nodeId, int vel)
  {
    int _velocity = 0;
    if( nodeId==0 )
    {
      unsigned short _ii;
      for(_ii=1;_ii<5;_ii++)
      {
        if( VCS_MoveWithVelocity(comHandler_, _ii, vel, errorCode_)!=0 )
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
      if( VCS_MoveWithVelocity(comHandler_, nodeId, vel, errorCode_)!=0 )
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

