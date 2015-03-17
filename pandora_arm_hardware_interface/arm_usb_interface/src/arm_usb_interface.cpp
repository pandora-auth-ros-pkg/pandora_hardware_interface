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
 * Author: Nikos Taras
 *********************************************************************/

#include "arm_usb_interface/arm_usb_interface.h"

namespace pandora_hardware_interface
{
namespace arm
{

ArmUsbInterface::ArmUsbInterface()
{
  openUsbPort();
}


ArmUsbInterface::~ArmUsbInterface()
{
  close(fd);
  ROS_INFO("[Head]: USB port closed because of program termination\n");
}

//  fcntl(fd, F_SETFL, FNDELAY);    //make read() non-blocking
//  fcntl(fd, F_SETFL, 0);  //make read() blocking
void ArmUsbInterface::openUsbPort()
{
  int timeout = 0;
  // To make read non-blocking use the following:
  // fd = open("/dev/head", O_RDWR | O_NOCTTY | O_NDELAY);
  while ((fd = open("/dev/head", O_RDWR | O_NOCTTY)) == -1)
  {
    ROS_ERROR("[Head]: cannot open USB port\n");
    ROS_ERROR("[Head]: open() failed with error [%s]\n", strerror(errno));

    ros::Duration(0.5).sleep();
    timeout++;
    if (timeout > 100)
      throw std::runtime_error("[Head]: cannot open USB port");
  }

  ROS_INFO("[Head]: USB port successfully opened\n");

  /*Needs some time to initialize, even though it opens succesfully.
   tcflush() didn't work without waiting at least 8 ms*/
  ros::Duration(0.03).sleep();

  // To save time you can see and change terminal settings in command line with stty command,
  // before implementing in software. Note: stty prefixes disabled flags with a dash.
  // See:  http://man7.org/linux/man-pages/man3/termios.3.html
  struct termios tios;

  if (tcgetattr(fd, &tios) < 0)
  {
    ROS_ERROR("init_serialport: Couldn't get term attributes\n");
  }

  tios.c_lflag &= ~(ICANON | ISIG | ECHO |  IEXTEN | ECHOE | ECHOK);

  tios.c_iflag &= ~(ICRNL | IXON);

  tios.c_oflag &= ~(OPOST);

  if (tcsetattr(fd, TCSANOW, &tios) < 0)
  {
    ROS_ERROR("init_serialport: Couldn't set term attributes\n");
  }
}


int ArmUsbInterface::readGrideyeValues(
  const char& grideyeSelect, uint8_t * values)
{
  int nr;
  uint8_t bufOUT;

  switch (grideyeSelect)
  {
    case 'C':
      bufOUT = COMMAND_GEYE_CENTER;
      break;
    case 'L':
      bufOUT = COMMAND_GEYE_LEFT;
      break;
    case 'R':
      bufOUT = COMMAND_GEYE_RIGHT;
      break;
    default:
      ROS_ERROR(
        "Undefined Grideye Selection! Left Grideye selected by  default.");
      bufOUT = COMMAND_GEYE_CENTER;
      break;
  }

  tcflush(fd, TCIOFLUSH);  // flushes both data received but not read,
                           // -> and data written but not transmitted

  nr = write(fd, (const void *)&bufOUT, COMMAND_NBYTES);
  if (nr != 1)
  {
    ROS_ERROR("[Head]: Write Error\n");
    reconnectUsb();
    return -1;
  }

  nr = read(fd, values, GEYE_NBYTES);  // blocking
  if (nr < 0)
  {
    ROS_ERROR("[Head]: Read Error\n");
    reconnectUsb();
    return -2;
  }
  else if (nr != GEYE_NBYTES)
  {
    ROS_ERROR("[Head]: Wrong number of bytes read\n");
    reconnectUsb();
    return -3;
  }
  else
  {
    /*
    std::stringstream ss;

    ss << "[Head]: " << grideyeSelect << " GridEYE = ";
    for (int i = 0; i < GEYE_NBYTES; ++i)
    {
      ss << static_cast<int>(values[i]) << " ";
    }
    ROS_INFO("%s", ss.str().c_str());
    */
    return 1;
  }
}


uint16_t ArmUsbInterface::readSonarValues(const char& sonarSelect)
{
  union
  {
    uint8_t sonarBufIN[SONAR_NBYTES];
    uint16_t sonarBufIN_uint16_t;
  };

  int nr;
  uint8_t bufOUT;

  switch (sonarSelect)
  {
    case 'L':
      bufOUT = COMMAND_SONAR_LEFT;
      break;
    case 'R':
      bufOUT = COMMAND_SONAR_RIGHT;
      break;
    default:
      bufOUT = COMMAND_SONAR_LEFT;
      ROS_ERROR("Undefined Sonar Selection! Left Sonar selected by default.");
      break;
  }

  tcflush(fd, TCIOFLUSH);  // flushes both data received but not read,
                           // -> and data written but not transmitted
  nr = write(fd, (const void *)&bufOUT, COMMAND_NBYTES);
  if (nr != 1)
  {
    ROS_ERROR("[Head]: Write Error\n");
    reconnectUsb();
    return -1;
  }
  nr = read(fd, sonarBufIN, SONAR_NBYTES);  // blocking
  if (nr < 0)
  {
    ROS_ERROR("[Head]: Read Error\n");
    reconnectUsb();
    return -2;
  }
  else if (nr != SONAR_NBYTES)
  {
    ROS_ERROR("[Head]: Wrong number of bytes read\n");
    reconnectUsb();
    return -3;
  }
  else
  {
    // ROS_INFO("[Head]: Sonar (%c) = %u\n", sonarSelect, sonarBufIN_uint16_t);
    return sonarBufIN_uint16_t;
  }
}


float ArmUsbInterface::readCo2Value()
{
  union
  {
    uint8_t CO2bufIN[CO2_NBYTES];
    float CO2bufIN_float;
  };

  int nr;
  uint8_t bufOUT;

  tcflush(fd, TCIOFLUSH);  // flushes both data received but not read,
                           // -> and data written but not transmitted

  bufOUT = COMMAND_CO2;
  nr = write(fd, (const void *)&bufOUT, COMMAND_NBYTES);
  if (nr != 1)
  {
    ROS_ERROR("[Head]: Write Error\n");
    reconnectUsb();
    return -1;
  }

  nr = read(fd, CO2bufIN, CO2_NBYTES);  // blocking
  if (nr < 0)
  {
    ROS_ERROR("[Head]: Read Error\n");
    reconnectUsb();
    return -2;
  }
  else if (nr != CO2_NBYTES)
  {
    ROS_ERROR("[Head]: Wrong number of bytes read\n");
    reconnectUsb();
    return -3;
  }
  else
  {
    // ROS_INFO("[Head]: CO2 Gas Reading = %f\n", CO2bufIN_float);
    return CO2bufIN_float;
  }
}

uint16_t ArmUsbInterface::readEncoderValue()
{
  union
  {
    uint8_t encoderBufIN[ENCODER_NBYTES];
    uint16_t encoderBufIN_uint16_t;
  };

  int nr;
  uint8_t bufOUT;

  tcflush(fd, TCIOFLUSH);  // flushes both data received but not read,
                           // -> and data written but not transmitted

  bufOUT = COMMAND_ENCODER;
  nr = write(fd, (const void *)&bufOUT, COMMAND_NBYTES);
  if (nr != 1)
  {
    ROS_ERROR("[Head]: Write Error\n");
    reconnectUsb();
    return -1;
  }

  nr = read(fd, encoderBufIN, ENCODER_NBYTES);  // blocking
  if (nr < 0)
  {
    ROS_ERROR("[Head]: Read Error\n");
    reconnectUsb();
    return -2;
  }
  else if (nr != ENCODER_NBYTES)
  {
    ROS_ERROR("[Head]: Wrong number of bytes read\n");
    reconnectUsb();
    return -3;
  }
  else
  {
    // ROS_INFO("[Head]: Encoder Reading = %u\n", encoderBufIN_uint16_t);
    return encoderBufIN_uint16_t;
  }
}

uint16_t ArmUsbInterface::readBatteryValues(const char& batterySelect)
{
  union
  {
    uint8_t batteryBufIN[BATTERY_NBYTES];
    uint16_t batteryBufIN_uint16_t;
  };

  int nr;
  uint8_t bufOUT;

  switch (batterySelect)
  {
    case 'M':
      bufOUT = COMMAND_BATTERY_MOTOR;
      break;
    case 'S':
      bufOUT = COMMAND_BATTERY_SUPPLY;
      break;
    default:
      ROS_ERROR(
        "Undefined Battery selection. Motor battery selected by default");
      bufOUT = COMMAND_BATTERY_MOTOR;  // shouldn't get in there
      break;
  }

  tcflush(fd, TCIOFLUSH);  // flushes both data received but not read,
                           // -> and data written but not transmitted

  nr = write(fd, (const void *)&bufOUT, COMMAND_NBYTES);
  if (nr != 1)
  {
    ROS_ERROR("[Head]: Write Error\n");
    reconnectUsb();
    return -1;
  }

  nr = read(fd, batteryBufIN, BATTERY_NBYTES);  // blocking
  if (nr < 0)
  {
    ROS_ERROR("[Head]: Read Error\n");
    reconnectUsb();
    return -2;
  }
  else if (nr != BATTERY_NBYTES)
  {
    ROS_ERROR("[Head]: Wrong number of bytes read\n");
    reconnectUsb();
    return -3;
  }
  else
  {
    // ROS_INFO("[Head]: Battery (%c) = %u\n", batterySelect, batteryBufIN_uint16_t);
    return batteryBufIN_uint16_t;
  }
}

void ArmUsbInterface::reconnectUsb()
{
  // reconnectUsb() should be called until communication is restored.
  close(fd);
  ROS_INFO("[Head]: USB port closed\n");
  ros::Duration(1.5).sleep();

  openUsbPort();

  ROS_INFO("[Head]: USB port reconnected successfully");
}

}  // namespace arm
}  // namespace pandora_hardware_interface
