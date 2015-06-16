/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
* Author:  Evangelos Apostolidis
* Author:  George Kouros
*********************************************************************/
#include "arduino_hardware_interface/arduino_hardware_interface.h"

namespace pandora_hardware_interface
{
namespace arduino
{
  ArduinoHardwareInterface::ArduinoHardwareInterface(
    ros::NodeHandle nodeHandle)
  :
    nodeHandle_(nodeHandle)
  {
    try
    {
      arduino_ = new ArduinoUsbInterface();
    }
    catch(std::exception& ex)
    {
      ROS_ERROR("%s", ex.what());
      exit(-1);
    }

    // connect and register battery interface
    registerBatteryInterface();

  }

  ArduinoHardwareInterface::~ArduinoHardwareInterface()
  {
    delete arduino_;
  }

  void ArduinoHardwareInterface::read()
  {
    uint16_t value;
    // ROS_INFO("Will read BATTERIES");
    // read voltage of batteries
    for (int ii = 0; ii < batteryName_.size(); ii++)
    {
        arduino_->readBatteryValues(batteryCode_[ii], &value);
        voltage_[ii] = value / 4096.0 * 33.0;
    }
  }


  void ArduinoHardwareInterface::registerBatteryInterface()
  {
    XmlRpc::XmlRpcValue batteryList;
    nodeHandle_.getParam("batteries", batteryList);
    ROS_ASSERT(
      batteryList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    batteryCode_ = new char[batteryList.size()];
    voltage_ = new double[batteryList.size()];

    std::vector<BatteryHandle>
      batteryHandle;
    std::string key;
    for (int ii = 0; ii < batteryList.size(); ii++)
    {
      ROS_ASSERT(
        batteryList[ii].getType() == XmlRpc::XmlRpcValue::TypeStruct);

      key = "name";
      ROS_ASSERT(
        batteryList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      batteryName_.push_back(static_cast<std::string>(batteryList[ii][key]));

      key = "code";
      ROS_ASSERT(
        batteryList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      batteryCode_[ii] = (static_cast<std::string>(batteryList[ii][key])).at(0);

      key = "max_voltage";
      ROS_ASSERT(
        batteryList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      voltage_[ii] = static_cast<double>(batteryList[ii][key]);

      BatteryHandle::Data data;
      data.name = batteryName_[ii];
      data.voltage = &voltage_[ii];
      batteryData_.push_back(data);
      BatteryHandle handle(batteryData_[ii]);
      batteryInterface_.registerHandle(handle);
    }
    registerInterface(&batteryInterface_);
  }



}  // namespace arduino
}  // namespace pandora_hardware_interface
