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
#ifndef EPOS_HANDLER_SERIAL_EPOS2_HANDLER_H
#define EPOS_HANDLER_SERIAL_EPOS2_HANDLER_H

#include "epos_handler/abstract_epos_handler.h"
#include "epos_handler/epos2_definitions.h"

namespace pandora_hardware_interface
{
namespace motor
{
  struct comInterface
  {
    char* deviceName;
    char* protocolStackName;
    char* interfaceName;
    char* portName;
    unsigned int baudrate;
    unsigned int timeout;
  };


  class SerialEpos2Handler: public AbstractEposHandler
  {
    public:
      SerialEpos2Handler( const std::string port, const unsigned int baudrate, const unsigned int timeout );
      virtual ~SerialEpos2Handler();
      virtual void getRPM( int* leftRearRpm, int* leftFrontRpm,
        int* rightRearRpm, int* rightFrontRpm );
      virtual void getCurrent( int* leftRearCurrent, int* leftFrontRpm,
        int* rightRearCurrent, int* rightFrontCurrent );
      virtual Error getError();
      virtual unsigned short writeRPM( const int leftRpm, const int rightRpm );
      bool eval_communicationInterface();
      void openDevice();
      void closeDevice();
      void resetDevice(unsigned short nodeId);

      /*!
       * @brief Changes the device's state to "Enable"
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      void setEnableState(unsigned short nodeId);

      /*!
       * @brief Changes the device's state to "Enable"
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      void setDisableState(unsigned short nodeId);


      /*!
       * @brief Checks if the device is Enabled
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      bool isEnableState(unsigned short nodeId);

      /*!
       * @brief Checks if the device is Disabled
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      bool isDisableState(unsigned short nodeId);

      /*!
       * @brief Changes the device state from "fault" to "disabled"
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      void clearFault(unsigned short nodeId);

      /*!
       * @brief Checks if the device is at "Fault" State
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       *
       * @TODO return Fault State errorCode
       */
      bool isFaultState(unsigned short nodeId);

      bool isQuickStopState(unsigned short nodeId);
      
      /*!
       * @brief Reads the state of the state machine
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       * @TODO Implement and return state index handler
       */
      unsigned short getState(unsigned short nodeId);

      //TODO --- Implement it to work with single nodeIds
      //0: ALL
      //1: nodeId#1
      //2: nodeId#2
      //...
      void getState_all(void);

      //TODO --- Implement it to work with single nodeIds
      //0: ALL
      //1: nodeId#1
      //2: nodeId#2
      //...
      void printState_all(void);

      /*!
       * @brief Changes the operation mode of an epos2 controller
       * to "Profile Velocity Mode"
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      void activate_profileVelocityMode(unsigned short nodeId);
           
      void moveWithVelocity(unsigned short nodeId, int vel);
    private:
      comInterface com_;
      void* comHandler_;
      unsigned int* errorCode_;
      unsigned short* nodeState_;
      
  };
}  // namespace motor
}  // namespace pandora_hardware_interface

#endif  // EPOS_HANDLER_SERIAL_EPOS2_HANDLER_H
