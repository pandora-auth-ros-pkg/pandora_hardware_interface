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
*********************************************************************/
#include "pandora_xmega_hardware_interface/xmega_hardware_interface.h"

namespace pandora_xmega_hardware_interface
{
  XmegaHardwareInterface::XmegaHardwareInterface(
    ros::NodeHandle nodeHandle)
  :
    nodeHandle_(nodeHandle),
    serialInterface(
      "/dev/ttyUSB0",
      115200,
      100)
  {
    serialInterface.init();
    // connect and register power supply interface
    registerPowerSupplyInterface();

    // connect and register power supply interface
    registerRangeSensorInterface();
  }

  XmegaHardwareInterface::~XmegaHardwareInterface()
  {
  }

  void XmegaHardwareInterface::read()
  {
    serialInterface.read();
    serialInterface.getBatteryData(
      reinterpret_cast<float*>(&voltage_[0]),
      reinterpret_cast<float*>(&voltage_[1]));

    std::map<int, pandora_xmega::RangeData> sensorMap;
    sensorMap = serialInterface.getRangeData();
    // How do I know if the sensor is IR or sonar
    std::string key;
    if (nodeHandle_.searchParam("i2c_map/1", key))
    {
      std::string val;
      nodeHandle_.getParam(key, val);
    }
    else
    {
      std::string string = string + "/sensors" + "1";
      nodeHandle_.setParam("i2c_map/1", string);

      rangeSensorName_[0] = string;
      pandora_xmega_hardware_interface::RangeSensorHandle handle(
          rangeData_[0]);
        rangeSensorInterface_.registerHandle(handle);
    }
  }

  void XmegaHardwareInterface::registerPowerSupplyInterface()
  {
    XmlRpc::XmlRpcValue powerSupplyNameList;
    nodeHandle_.getParam("power_supplies/name", powerSupplyNameList);
    ROS_ASSERT(
      powerSupplyNameList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    XmlRpc::XmlRpcValue maxVoltageList;
    nodeHandle_.getParam("power_supplies/max_voltage", maxVoltageList);
    ROS_ASSERT(
      maxVoltageList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    std::vector<pandora_xmega_hardware_interface::PowerSupplyHandle>
      powerSupplyHandle;
    for (int ii = 0; ii < powerSupplyNameList.size(); ii++)
    {
      ROS_ASSERT(
        powerSupplyNameList[ii].getType() == XmlRpc::XmlRpcValue::TypeString);
      powerSupplyNames_.push_back(
        static_cast<std::string>(powerSupplyNameList[ii]));

      ROS_ASSERT(
        maxVoltageList[ii].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      voltage_.push_back(
        static_cast<double>(maxVoltageList[ii]));

      pandora_xmega_hardware_interface::PowerSupplyHandle::Data data;
      data.name = powerSupplyNames_[ii];
      data.voltage = &voltage_[ii];
      powerSupplyData_.push_back(data);
      pandora_xmega_hardware_interface::PowerSupplyHandle handle(
        powerSupplyData_[ii]);
      powerSupplyInterface_.registerHandle(handle);
    }
    registerInterface(&powerSupplyInterface_);
  }

  void XmegaHardwareInterface::registerRangeSensorInterface()
  {
    XmlRpc::XmlRpcValue rangeSensorNameList;
    nodeHandle_.getParam("range_sensors/name", rangeSensorNameList);
    ROS_ASSERT(
      rangeSensorNameList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    XmlRpc::XmlRpcValue frameIdList;
    nodeHandle_.getParam("range_sensors/frame_id", frameIdList);
    ROS_ASSERT(
      frameIdList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    XmlRpc::XmlRpcValue radiationTypeList;
    nodeHandle_.getParam("range_sensors/radiation_type", radiationTypeList);
    ROS_ASSERT(
      radiationTypeList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    XmlRpc::XmlRpcValue fieldOfViewList;
    nodeHandle_.getParam("range_sensors/field_of_view", fieldOfViewList);
    ROS_ASSERT(
      fieldOfViewList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    XmlRpc::XmlRpcValue minRangeList;
    nodeHandle_.getParam("range_sensors/min_range", minRangeList);
    ROS_ASSERT(
      minRangeList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    XmlRpc::XmlRpcValue maxRangeList;
    nodeHandle_.getParam("range_sensors/max_range", maxRangeList);
    ROS_ASSERT(
      maxRangeList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    // We use size()-1, because the last value in the list is used for the
    // default options for an unknown range sensor
    for (int ii = 0; ii < rangeSensorNameList.size(); ii++)
    {
      ROS_ASSERT(
        rangeSensorNameList[ii].getType() == XmlRpc::XmlRpcValue::TypeString);
      rangeSensorName_.push_back(
        static_cast<std::string>(rangeSensorNameList[ii]));

      ROS_ASSERT(
        frameIdList[ii].getType() == XmlRpc::XmlRpcValue::TypeString);
      frameId_.push_back(
        static_cast<std::string>(frameIdList[ii]));

      ROS_ASSERT(
        radiationTypeList[ii].getType() == XmlRpc::XmlRpcValue::TypeInt);
      radiationType_.push_back(
        static_cast<int>(radiationTypeList[ii]));

      ROS_ASSERT(
        fieldOfViewList[ii].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      fieldOfView_.push_back(
        static_cast<double>(fieldOfViewList[ii]));

      ROS_ASSERT(
        minRangeList[ii].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      minRange_.push_back(
        static_cast<double>(minRangeList[ii]));

      ROS_ASSERT(
        maxRangeList[ii].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      maxRange_.push_back(
        static_cast<double>(maxRangeList[ii]));
      boost::array<double, 5> initialRange;
      initialRange[0] = initialRange[1] =
        initialRange[2] = initialRange[3] =
        initialRange[4] = static_cast<double>(maxRangeList[ii]);
      range_.push_back(initialRange);
      bufferCounter_.push_back(0);

      pandora_xmega_hardware_interface::RangeSensorHandle::Data data;
      data.name = rangeSensorName_[ii];
      data.frameId = frameId_[ii];
      data.radiationType = &radiationType_[ii];
      data.fieldOfView = &fieldOfView_[ii];
      data.minRange = &minRange_[ii];
      data.maxRange = &maxRange_[ii];
      data.range = &range_[ii];
      rangeData_.push_back(data);
      if (ii > 0)
      {
        pandora_xmega_hardware_interface::RangeSensorHandle handle(
          rangeData_[ii]);
        rangeSensorInterface_.registerHandle(handle);
      }
    }
    registerInterface(&rangeSensorInterface_);
  }

}  // namespace pandora_xmega_hardware_interface
