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

  #define RIGHT_FRONT_MOTOR 1
  #define RIGHT_REAR_MOTOR  2
  #define LEFT_FRONT_MOTOR  3 
  #define LEFT_REAR_MOTOR   4 
  
  struct comInterface
  {
    char* deviceName;
    char* protocolStackName;
    char* interfaceName;
    char* portName;
    uint32_t baudrate;
    uint32_t timeout;
  };


  class SerialEpos2Handler: public AbstractEposHandler
  {
    private:
      comInterface com_;
      void* comHandler_;
      uint32_t* errorCode_;
      uint16_t* nodeState_;
    public:
      SerialEpos2Handler(const std::string port, const uint32_t baudrate, const uint32_t timeout);
      virtual ~SerialEpos2Handler();
      virtual void getRPM(int* leftRearRpm, int* leftFrontRpm,
        int* rightRearRpm, int* rightFrontRpm);
      virtual void getCurrent(int* leftRearCurrent, int* leftFrontCurrent,
        int* rightRearCurrent, int* rightFrontCurrent);
      virtual Error getError();
      virtual uint16_t writeRPM(const int leftRpm, const int rightRpm);
      bool eval_communicationInterface();
      void openDevice();
      void closeDevice();
      void resetDevice(uint16_t nodeId);

      /*!
       * @brief Changes the device's state to "Enable"
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      void setEnableState(uint16_t nodeId);

      /*!
       * @brief Changes the device's state to "Enable"
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      void setDisableState(uint16_t nodeId);


      /*!
       * @brief Checks if the device is Enabled
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      bool isEnableState(uint16_t nodeId);

      /*!
       * @brief Checks if the device is Disabled
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      bool isDisableState(uint16_t nodeId);

      /*!
       * @brief Changes the device state from "fault" to "disabled"
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      void clearFault(uint16_t nodeId);

      /*!
       * @brief Checks if the device is at "Fault" State
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       *
       * @TODO return Fault State errorCode
       */
      bool isFaultState(uint16_t nodeId);

      bool isQuickStopState(uint16_t nodeId);
      
      /*!
       * @brief Reads the state of the state machine
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       * @TODO Implement and return state index handler
       */
      uint16_t getState(uint16_t nodeId);

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
       *
       *  If nodeId = 0 then command trasmits to everyone.
       */
      void activate_profileVelocityMode(uint16_t nodeId);
           
      /*!
       * @brief Commands target velocity
       * param nodeId NodeID of the epos2 controller defined on CAN-Bus
       *
       *  If nodeId = 0 then command trasmits to everyone.
       */
      void moveWithVelocity(uint16_t nodeId, int vel);

      /*!
       * @brief Reads the velocity actual value 
       * param nodeId NodeID of the epos2 controller defined on CAN-Bus
       *
       *  If nodeId = 0 then command trasmits to everyone.
       */
      void read_velocityActual(uint16_t nodeId, int32_t* velActual);

      /*!
       * @brief Reads the velocity average value 
       * param nodeId NodeID of the epos2 controller defined on CAN-Bus
       *
       *  If nodeId = 0 then command trasmits to everyone.
       */
      void read_velocityAvg(uint16_t nodeId, int32_t* velAvg);

      /*!
       * @brief Reads the current actual value 
       * param nodeId NodeID of the epos2 controller defined on CAN-Bus
       *
       *  If nodeId = 0 then command trasmits to everyone.
       */
      void read_currentActual(uint16_t nodeId, int16_t* currentActual); 

      /*!
       * @brief Reads the current average value 
       * param nodeId NodeID of the epos2 controller defined on CAN-Bus
       *
       *  If nodeId = 0 then command trasmits to everyone.
       */
      void read_currentAvg(uint16_t nodeId, int16_t* currentAvg); 

  };
}  // namespace motor
}  // namespace pandora_hardware_interface

#endif  // EPOS_HANDLER_SERIAL_EPOS2_HANDLER_H
