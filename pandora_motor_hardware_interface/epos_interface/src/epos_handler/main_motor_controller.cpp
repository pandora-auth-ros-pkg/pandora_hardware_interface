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
#include "epos_handler/main_motor_controller.h"


std::string MainMotorController::convertStatusToString(epos::CommandStatus st) {
  if(st == epos::SUCCESS)       return "SUCCESS";
  else if (st == epos::BUSY)    return "BUSY";
  else if (st == epos::NACK)    return "NACK";
  else if (st == epos::TIMEOUT) return "TIMEOUT";
  else if (st == epos::RS232)   return "RS232";
  else if (st == epos::API)     return "API";
  else if (st == epos::RESYNC)  return "RESYNC";
  else if (st == epos::PROTOCOL)return "PROTOCOL";
  else                        return "INVALID. Check Code";
}


MainMotorController::MainMotorController  (std::string dev,
    int speed,
    int timeout,
    ros::NodeHandle &handle,
    float period)
{
  motors = new EposHandler(dev, speed, timeout);
  lastSetSpeedCall = ros::Time::now();
  falseSpeedCall = false;
  timer = m_handle.createTimer(
            ros::Duration(period),
            &MainMotorController::postStatus,
            this);
  softBusy = false;
  m_handle = handle;
  twistSub = m_handle.subscribe("/cmd_vel", 1, &MainMotorController::setVelocityCallback, this);
//~     main_motor_measurements= m_handle.advertise<main_motor_control_communications::motor_rpm_msg>("rpm_topic",4);
  _updater.setHardwareID("EPOS");
  _updater.add("Motor Speed", this, &MainMotorController::speedDiagnostic);
  _updater.add("Motor Status", this, &MainMotorController::statusDiagnostic);
}

MainMotorController::~MainMotorController() {
  delete motors;
}

void MainMotorController::postStatus(const ros::TimerEvent& event) {
  guard.lock();
  if (!calledTooOften()) {
    Kinematic::RPM actualRPM = motors->getRPM();
//~         main_motor_control_communications::motor_rpm_msg msg;
//~         msg.rpm_left_demand=controlInput.left;
//~         msg.rpm_right_demand=controlInput.right;
//~         msg.rpm_left_actual=actualRPM.left;
//~         msg.rpm_right_actual=actualRPM.right;
//~         actualRPM=motors->getRPM();
//~         main_motor_measurements.publish(msg);
    _updater.update();
  }
  guard.unlock();
  return;
}

epos::CommandStatus MainMotorController::setSpeed(float linear, float angular) {
  epos::CommandStatus retVal;
  guard.lock();
  if(calledTooOften()) {
    ROS_DEBUG("[mainMotorControl]: setVehicleSpeed called too frequently. Cannot respond!");
    retVal = epos::BUSY;
  }

  ROS_INFO("mainMotorControl: got speed, linear: '%f', angular: '%f'",
           linear, angular);

  //Robot kinematic velocity structure
  Kinematic::Velocity velocity(linear, 0, -angular);

  //Get required robot rpms

  controlInput = calculateRPM(velocity);
  motorStatus = motors->writeRPM(controlInput);
  if(motorStatus != epos::SUCCESS) ROS_ERROR("error setting speed ");
  retVal = motorStatus;
  guard.unlock();
  return retVal;

}

void MainMotorController::setVelocityCallback(const geometry_msgs::Twist& msg)
{

  epos::CommandStatus ret = setSpeed(msg.linear.x, -msg.angular.z);
  if(ret != 0 && !softBusy) {
    ROS_ERROR_STREAM("MainMotorCOntroller::setSpeed: Error" << convertStatusToString(ret));

  }
  else if (softBusy) softBusy = false;
  return ;
}


void MainMotorController::speedDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if (!falseSpeedCall) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Vehicle Speed OK");
  }
  else {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                  "Set Vehicle Speed Cannot Respond, called too frequently ");
  }
}

void MainMotorController::statusDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if(motorStatus == 0) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motor Status == SUCCESS");
  }
  else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor Status == " + convertStatusToString(motorStatus));
  }
}



bool MainMotorController::calledTooOften() {
  bool retVal;
  timeBetweenCalls = ros::Time::now() - lastSetSpeedCall;
  if(timeBetweenCalls < ros::Duration(.1)) {
    ROS_DEBUG("[mainMotorControl]: setVehicleSpeed called too frequently. Cannot respond!");
    falseSpeedCall = true;
    softBusy = true;
    retVal = true;
  }
  else {
    falseSpeedCall = false;
    lastSetSpeedCall = ros::Time::now();
    retVal = false;
  }
  return retVal;
}


