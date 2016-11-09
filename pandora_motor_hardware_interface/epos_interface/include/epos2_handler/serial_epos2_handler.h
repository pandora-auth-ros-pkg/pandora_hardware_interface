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
#ifndef EPOS2_HANDLER_SERIAL_EPOS2_HANDLER_H
#define EPOS2_HANDLER_SERIAL_EPOS2_HANDLER_H

#include "epos_handler/abstract_epos_handler.h"
#include "epos2_gateway/epos2_gateway.h"

namespace pandora_hardware_interface
{
namespace motor
{

  struct Epos2Controller
  {
    std::string name_;
    uint16_t nodeId_;
    uint32_t errorCode_;
    std::string motorId_;
    uint16_t state_;
    int32_t rpm_;
    int16_t current_;
  };


  // ====================Serial Epos2 Handler Class========================

  class SerialEpos2Handler
  {
    private:
      /*! NodeHandler under private namespace "~/epos2config"*/
      ros::NodeHandle epos2_nh_;
      boost::scoped_ptr<Epos2Gateway> epos2Gateway_;
      std::vector<Epos2Controller> epos2Controllers_;
      std::map<std::string, uint16_t> nameToIndexMap_;
      double currentToTorqueMultiplier_;
      uint16_t gatewayId_;

      int connectionAttempts_;

      /*
       * operationMode_ = 0 ==> velocity mode
       * operationMode_ = 1 ==> current mode
       */
      int operationMode_;

    public:
      /*!
       * @brief Constructor
       */
      SerialEpos2Handler(void);

      /*!
       * @brief Destructor
       */
      virtual ~SerialEpos2Handler();

      /*!
       * @brief Reads current velocity (rpm) from motor controllers
       * @details The rpm is also stored in the motor controller object
       * @param name [std::string] : name of motor controller
       * @param rpm [int*] : ptr to container to store the rpm
       * @return Void
       */
      virtual void getRPM(const std::string name, int* const rpm);

      /*!
       * @brief Reads output current (mA) from motor controllers
       * @details The current is also stored in the motor controller object
       * @param name [std::string] : name of motor controller
       * @param current [int*] : ptr to container to store the current
       * @return Void
       */
      virtual void getCurrent(const std::string name, int* const current);


      /*!
       * @brief Read error from motor controllers
       * @return Error
       */
      virtual Error getError();


      /*!
       * @brief Writes velocity commands (rpm) to motor cotrollers
       * @param name [std::string] : name of motor controller
       * @param rpm [const int] : rpm to write to controller
       * @return Void
       */
      virtual uint16_t writeRPM(const std::string name, const int rpm);


      /*!
       * @brief Reads motor controller states and stores the values
       *  in a private scope
       */
      void readStates(void);


      /*!
       * @brief Read the state of all motor controllers
       */
      void stateHandle(void);


      /*!
       * Read current of motor, convert to torque and return it
       */
      void  getTorque(const std::string name, double* const torque);


      /*!
       * @brief Converts single WHEEL Torque to motor current
       */
      int16_t torqueToCurrent(double _input_torque);


      /*!
       * @brief Converts single motor current to WHEEL Torque
       */
      double currentToTorque(int _input_current);


      /*!
       * @brief Writes torque to motor controller
       * @param name [std::string] : name of motor controller
       * @param torque [double] : motor controller torque command
       */
      uint16_t writeTorque(const std::string name, const double torque);


      /*!
       * @brief Switches between current and velocity mode
       * @param mode [int] : 0=velocity_mode, 1=current_mode
       */
      void setMode(int mode);


      /*!
       * @brief returns the operation mode
       */
      int getMode(void);
  };
}  // namespace motor
}  // namespace pandora_hardware_interface
#endif  // EPOS2_HANDLER_SERIAL_EPOS2_HANDLER_H
