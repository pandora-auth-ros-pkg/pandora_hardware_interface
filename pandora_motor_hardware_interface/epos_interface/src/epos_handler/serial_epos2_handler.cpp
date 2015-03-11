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
    //com_.portName = port.c_str();
    com_.baudrate = baudrate;
    com_.timeout = timeout;
    openDevice();
  }

  SerialEpos2Handler::~SerialEpos2Handler()
  {
  
  }

  unsigned int SerialEpos2Handler::openDevice()
  {
    unsigned int* pErrorCode;
    com_.comHandler = VCS_OpenDevice(com_.deviceName, com_.protocolStackName,
      com_.interfaceName, com_.portName, pErrorCode);
    return *pErrorCode;
  }

  void SerialEpos2Handler::getRPM(int* leftRearRpm, int* leftFrontRpm,
    int* rightRearRpm, int* rightFrontRpm)
  {
  
  }
  void SerialEpos2Handler::getCurrent( int* leftRearCurrent, int* leftFrontRpm,
    int* rightRearCurrent, int* rightFrontCurrent )
  {
  
  }

  Error SerialEpos2Handler::getError() {}
  unsigned short SerialEpos2Handler::writeRPM( const int leftRpm, const int rightRpm ) {}



}  // namespace motor
}  // namespace pandora_hardware_interface

