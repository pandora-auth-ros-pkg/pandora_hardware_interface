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
*
*********************************************************************/

#include "epos2_handler/serial_epos2_handler.h"

namespace pandora_hardware_interface
{
namespace motor
{
  SerialEpos2Handler::SerialEpos2Handler():
    epos2_nh_("/motor/epos2")
  {
    std::string _portName, _deviceName, _protocolStackName, _interfaceName;
    int _baudrate, _timeout, _gatewayId;
    std::vector<int> _nodeIds;
    std::vector<std::string> _controllerNames;

    // Load epos2 interface configurations from parameter server
    epos2_nh_.param<std::string>("interface/portName", _portName, "");
    epos2_nh_.param<int>("interface/baudrate", _baudrate, 0);
    epos2_nh_.param<int>("interface/timeout", _timeout, 0);
    epos2_nh_.param<std::string>("interface/deviceName", _deviceName, "");
    epos2_nh_.param<std::string>("interface/protocolStackName",
      _protocolStackName, "");
    epos2_nh_.param<std::string>("interface/interfaceName", _interfaceName,
      "");
    epos2_nh_.param<int>("interface/connection_attempts",
      connectionAttempts_, 10);

    // Load epos2 controllers' configuration from the parameter server
    epos2_nh_.param<int>("controllers/epos2Gateway_id", _gatewayId, 1);
    epos2_nh_.getParam("controllers/names", _controllerNames);
    epos2_nh_.getParam("controllers/node_ids", _nodeIds);
    epos2_nh_.param<double>("controllers/current_to_torque_multiplier",
      currentToTorqueMultiplier_, 0);

    // check if loaded parameters are valid
    if (_controllerNames.size() == 0 || _nodeIds.size() == 0)
    {
      ROS_ERROR(
        "[Epos2-Handler] failed to load controllers from param server!");
      exit(1);
    }
    else if (_controllerNames.size() != _nodeIds.size())
    {
      ROS_ERROR(
        "[Epos2-Handler] number of controller names and ids don't match");
    }
    else
    {
      ROS_INFO(
        "[Epos2-Handler] controllers loaded successfully! Controller List:");
      for (int ii = 0; ii < _controllerNames.size(); ii++)
      {
        ROS_INFO(
          "[Epos2-Handler] Controller[%d]: %s",
          ii, _controllerNames[ii].c_str());
      }
    }

    for (int ii = 0; ii < _controllerNames.size(); ii++)
    {
      Epos2Controller epos2Controller;
      epos2Controller.name_ = _controllerNames[ii];
      epos2Controller.nodeId_ = static_cast<uint16_t>(_nodeIds[ii]);
      epos2Controllers_.push_back(epos2Controller);
      nameToIndexMap_.insert(
        std::pair<std::string, int>(_controllerNames[ii], ii));
    }

    epos2Gateway_.reset(new Epos2Gateway(_portName, _baudrate, _timeout,
        _deviceName, _protocolStackName, _interfaceName));

    gatewayId_ = static_cast<uint16_t>(_gatewayId);

    int retries = 0;

    // Initiate Communication with epos2 gateway
    while ( retries < connectionAttempts_)
    {
      if ( epos2Gateway_->openDevice() )
      {
        ROS_INFO("[Epos2-Handler]: Communication Success");
        break;
      }
      else
      {
        ROS_WARN("[Epos2-Handler]: Communication Failed!");
        retries++;
        ros::Duration(1).sleep();  // sleep for a second
      }
    }

    if (retries >= connectionAttempts_)
    {
      ROS_ERROR("[Epos2-Handler] Failed to connect to device! Exiting...");
      exit(1);
    }

    // Initialize motor controller states {Enabled}
    readStates();   // Read current state of the Motors
    stateHandle();  // Calls the state handler to handle the states

    // Set every epos2 controller at profileVelocityMode on startup
    for (uint16_t _ii = 0; _ii < epos2Controllers_.size() ; _ii++)
      epos2Gateway_->activate_profileVelocityMode(epos2Controllers_[_ii].nodeId_);


    // Setting operation mode to velocity_mode
    operationMode_ = 0;
  }


  SerialEpos2Handler::~SerialEpos2Handler()
  {
    // brake all motors and then set them to disable state
    for (uint16_t _ii = 0; _ii < epos2Controllers_.size() ; _ii++)
    {
      epos2Gateway_->set_targetVelocity(epos2Controllers_[_ii].nodeId_, 0);
      epos2Gateway_->setDisableState(epos2Controllers_[_ii].nodeId_);
    }

    // terminate communication with gateway
    epos2Gateway_->closeDevice();
  }


  void SerialEpos2Handler::stateHandle(void)
  {
    for (uint16_t _ii = 0; _ii < epos2Controllers_.size(); _ii++)
    {
      switch (epos2Controllers_[_ii].state_)
      {
        case 0:  // DISABLE STATE
          epos2Gateway_->setEnableState(epos2Controllers_[_ii].nodeId_);
          break;
        case 1:  // ENABLE STATE
          break;
        case 2:  // QUICKSTOP STATE
          epos2Gateway_->setEnableState(epos2Controllers_[_ii].nodeId_);
          break;
        case 3:  // FAULTY STATE
          epos2Gateway_->clearFaultState(epos2Controllers_[_ii].nodeId_);
          epos2Gateway_->setEnableState(epos2Controllers_[_ii].nodeId_);
          break;
        case 9:  // CANNOT COMMUNICATE
          if (epos2Controllers_[_ii].nodeId_ == gatewayId_)
          {
            // Cannot communicate with epos2-Gateway
            ROS_WARN(
              "[Epos2-Handler]: Cannot communicate with epos2-Gateway.");
          }
          epos2Gateway_->resetNode(epos2Controllers_[_ii].nodeId_);
          epos2Gateway_->setEnableState(epos2Controllers_[_ii].nodeId_);
          break;
        default:  // UKNOWN STATE
          ROS_WARN("[Epos2-Handler]: UNKNOWN STATE --> [%d]",
            epos2Controllers_[_ii].state_);
          epos2Gateway_->resetNode(epos2Controllers_[_ii].nodeId_);
          epos2Gateway_->setEnableState(epos2Controllers_[_ii].nodeId_);
          break;
      }
    }

    readStates();

    for (uint16_t _ii = 0; _ii < epos2Controllers_.size(); _ii++)
      if (epos2Controllers_[_ii].state_ != 1)
        stateHandle();
  }


  void SerialEpos2Handler::getRPM(const std::string name, int* const rpm)
  {
    epos2Gateway_->read_velocityAvg(
      epos2Controllers_[nameToIndexMap_[name]].nodeId_,
      &epos2Controllers_[nameToIndexMap_[name]].rpm_);

    *rpm = epos2Controllers_[nameToIndexMap_[name]].rpm_;
  }


  void SerialEpos2Handler::getCurrent(
    const std::string name, int* const current)
  {
    epos2Gateway_->read_currentAvg(
      epos2Controllers_[nameToIndexMap_[name]].nodeId_,
      &epos2Controllers_[nameToIndexMap_[name]].current_);

    *current = epos2Controllers_[nameToIndexMap_[name]].current_;
  }


  Error SerialEpos2Handler::getError()
  {
    //TODO(gkouros/klpanagi): implement getError method
  }

  uint16_t SerialEpos2Handler::writeRPM(const std::string name, const int rpm)
  {
    ROS_DEBUG("[Epos2-Handler]: Setting speed %d to %s", rpm, name.c_str());
    epos2Gateway_->set_targetVelocity(
      epos2Controllers_[nameToIndexMap_[name]].nodeId_, rpm);
    return 1;
  }


  void SerialEpos2Handler::readStates(void)
  {
    for (uint16_t _ii = 0; _ii < epos2Controllers_.size(); _ii++)
    {
      epos2Gateway_->readState(epos2Controllers_[_ii].nodeId_,
        &epos2Controllers_[_ii].state_);

      ROS_INFO("[Epos2-Handler]: NodeId-%d State==%d",
        epos2Controllers_[_ii].nodeId_, epos2Controllers_[_ii].state_);
    }
  }


  void SerialEpos2Handler::getTorque(
    const std::string name, double* const torque)
  {
    int _tempCurrent;
    // get current of motor controller
    getCurrent(
      epos2Controllers_[nameToIndexMap_[name]].name_, &_tempCurrent);
      epos2Controllers_[nameToIndexMap_[name]].current_ =
        static_cast<int16_t>(_tempCurrent);

    // convert current to torque
    *torque =
      currentToTorque(epos2Controllers_[nameToIndexMap_[name]].current_);
  }


  int16_t SerialEpos2Handler::torqueToCurrent(double _input_torque)
  {
    return static_cast<int16_t>(_input_torque / currentToTorqueMultiplier_);
  }


  double SerialEpos2Handler::currentToTorque(int _input_current)
  {
    return static_cast<double>(_input_current * currentToTorqueMultiplier_);
  }


  uint16_t SerialEpos2Handler::writeTorque(
    const std::string name, const double torque)
  {
    // Step I :Convert Torques to currents
    int16_t current = torqueToCurrent(torque);

    // Step II: Send commands to motors
    ROS_DEBUG("[Epos2-Handler]: Setting torque to controller %s: %f",
      name.c_str(), torque);

    epos2Gateway_->set_targetCurrent(
      epos2Controllers_[nameToIndexMap_[name]].nodeId_, current);

    return 1;
  }

  void SerialEpos2Handler::setMode(int mode)
  {
    // activate profile velocity or current mode for all motor controllers
    switch (mode)
    {
      case 0:
        // Activate Velocity Mode
        ROS_INFO("[Epos2-Handler]: Entering Velocity Mode");
        for (uint16_t _ii = 0; _ii < epos2Controllers_.size(); _ii++)
          epos2Gateway_->activate_profileVelocityMode(
            epos2Controllers_[_ii].nodeId_);
        break;
      case 1:
        // Activate Current Mode
        ROS_INFO("[Epos2-Handler]: Entering Current Mode");
        for (uint16_t _ii = 0; _ii < epos2Controllers_.size(); _ii++)
          epos2Gateway_->activate_currentMode(
            epos2Controllers_[_ii].nodeId_);
        break;
      default:
        ROS_WARN("There is no such state");
        break;
    }
    // store operation mode in corresponding member variable
    operationMode_ = mode;
  }

  int SerialEpos2Handler::getMode(void)
  {
    return operationMode_;
  }

}  // namespace motor
}  // namespace pandora_hardware_interface
