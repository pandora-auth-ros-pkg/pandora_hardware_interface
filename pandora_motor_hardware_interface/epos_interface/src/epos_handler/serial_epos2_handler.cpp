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
    com_.deviceName = "EPOS2";
    com_.protocolStackName = "MAXON_RS232";
    com_.interfaceName = "RS232";
    strcpy( com_.portName, port.c_str() );
    com_.baudrate = baudrate;
    com_.timeout = timeout;
    openDevice();
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
    comHandler_ = VCS_OpenDevice(com_.deviceName, com_.protocolStackName,
      com_.interfaceName, com_.portName, errorCode_);
    if ( comHandler_==0 || errorCode_!=0 )
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
    if( VCS_GetProtocolStackSettings(comHandler_, &_baudrate, &_timeout, errorCode_ )!=0 )
    {
      if( VCS_SetProtocolStackSettings(comHandler_, com_.baudrate, com_.timeout, errorCode_)!=0 )
      {
        if( VCS_GetProtocolStackSettings(comHandler_, &_baudrate, &_timeout, errorCode_)!=0 )
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
      //TODO --- Resolve errorCode for getEnableState function call 
    }

  }

  Error SerialEpos2Handler::getError() {}
  unsigned short SerialEpos2Handler::writeRPM( const int leftRpm, const int rightRpm ) {}



}  // namespace motor
}  // namespace pandora_hardware_interface

