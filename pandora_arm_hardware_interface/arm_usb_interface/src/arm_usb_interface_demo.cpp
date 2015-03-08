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
 * Author: Orestis Zachariadis
 *********************************************************************/
#include "arm_usb_interface/arm_usb_interface.h"

int main(int argc, char** argv)
{
  ros::Time::init();

  pandora_hardware_interface::arm::ArmUsbInterface arm;

  double voltage = 0;

  while (1)
  {
    ROS_INFO("Left Sonar measurement: %d", arm.readSonarValues('L'));
    ROS_INFO("Riht Sonar measurement: %d", arm.readSonarValues('R'));
    ROS_INFO("CO2 measurement: %f", arm.readCo2Value());
    ROS_INFO("Encoder measurement: %d", arm.readEncoderValue());

    uint8_t temperature[64];

    arm.readGrideyeValues('C', temperature);
    for (int ii = 0; ii < 64; ii++)
    {
      if (temperature[ii] < 0 || temperature[ii] > 255)
      {
        ROS_INFO("GridEye values were not read correctly.");
        break;
      }
      else if (ii=63)
        ROS_INFO("GridEye values were read correctly.");
    }

    voltage = arm.readBatteryValues('M');
    voltage = (voltage/4096.)*33;
    ROS_INFO("Motor Voltage measurement: %f", voltage);

    voltage = arm.readBatteryValues('S');
    voltage = (voltage/4096.)*33;
    ROS_INFO("Supply Voltage measurement %f", voltage);

    ros::Duration(0.5).sleep();
  }
  return 0;
}

