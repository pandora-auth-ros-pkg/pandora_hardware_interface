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

#include <iostream>

#include <cstdio>

#include "ros/ros.h"


#include <epos_gateway/epos_serial_gateway.h>
#include <epos_handler/kinematic.h>
#include <fstream>


#include "diagnostic_updater/diagnostic_updater.h"

#include "epos_handler/main_motor_controller.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "epos");
  ros::NodeHandle n;
  double period;
  if (!n.getParam("/mainMotorControl/statusPostPeriod", period)) {
    ROS_ERROR("statusPostPeriod not set. Setting to 0.5");
    period = 0.5;
  }
  MainMotorController controller("/dev/ttyS0", 115200, 500, n, period);
  ros::spin();
  return 0;
}

