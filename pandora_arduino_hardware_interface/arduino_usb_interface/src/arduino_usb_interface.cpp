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

#include "arduino_usb_interface/arduino_usb_interface.h"

namespace pandora_hardware_interface
{
namespace arduino
{

  ArduinoUsbInterface::ArduinoUsbInterface()
  {
    openUsbPort();
  }


  ArduinoUsbInterface::~ArduinoUsbInterface()
  {
    close(fd);
    ROS_INFO("[arduino]: USB port closed because of program termination\n");
  }


  //  fcntl(fd, F_SETFL, FNDELAY);    //make read() non-blocking
  //  fcntl(fd, F_SETFL, 0);  //make read() blocking
  void ArduinoUsbInterface::openUsbPort()
  {
    int timeout = 0;
    // To make read non-blocking use the following:
    // fd = open("/dev/arduino", O_RDWR | O_NOCTTY | O_NDELAY);
    while ((fd = open("/dev/arduino", O_RDWR | O_NOCTTY | O_NDELAY)) == -1)
    {
      ROS_ERROR("[arduino]: cannot open USB port\n");
      ROS_ERROR("[arduino]: open() failed with error [%s]\n", strerror(errno));

      ros::Duration(0.5).sleep();

      if (timeout > 100)
        throw std::runtime_error("[arduino]: cannot open USB port");
      else
        timeout++;
    }

    ROS_INFO("[arduino]: USB port successfully opened\n");

    // Needs some time to initialize, even though it opens succesfully.
    // tcflush() didn't work without waiting at least 8 ms
    ros::Duration(0.03).sleep();

    // To save time you can see and change terminal settings in command line with stty command,
    // before implementing in software. Note: stty prefixes disabled flags with a dash.
    // See:  http://man7.org/linux/man-pages/man3/termios.3.html
    struct termios tios;

    if (tcgetattr(fd, &tios) < 0)
      ROS_ERROR("init_serialport: Couldn't get term attributes\n");

    tios.c_lflag &= ~(ICANON | ISIG | ECHO |  IEXTEN | ECHOE | ECHOK);

    tios.c_iflag &= ~(ICRNL | IXON);

    tios.c_oflag &= ~(OPOST);

    tios.c_cc[VTIME] = 1;  // set timeout to 100ms

    if (tcsetattr(fd, TCSANOW, &tios) < 0)
      ROS_ERROR("init_serialport: Couldn't set term attributes\n");
  }

  int ArduinoUsbInterface::readData(
    int fd, uint8_t bufOut, uint8_t read_bytes, uint8_t* readBuf)
  {
    // flush both data received but not read and data written but not transmitted
    tcflush(fd, TCIOFLUSH);
    // ROS_INFO("JUST FLUSHED BUFFER");

    fd_set set;
    struct timeval timeout;
    int rv;
    uint8_t buff[100];
    int len = 100;

    FD_ZERO(&set); /* clear the set */
    FD_SET(fd, &set); /* add our file descriptor to the set */

    timeout.tv_sec = 3;
    timeout.tv_usec = 0;

    int nr;

    nr = write(fd, &bufOut, 1);
    if (nr != 1)
    {
      ROS_ERROR("[arduino]: Write Error\n");
      reconnectUsb();
      return WRITE_ERROR;
    }

    
    for (int i = 0; i < read_bytes; i++)
    {
      rv = select(fd + 1, &set, NULL, NULL, &timeout);
      if (rv == -1)
      {
        ROS_ERROR("select error!"); /* an error accured */
        return SELECT_ERROR;
      }
      else if (rv == 0)
      {
        ROS_INFO("timeout!"); /* a timeout occured */
        return READ_TIMEOUT;
      }
      // ROS_INFO("Before BUFFER READ");
      nr = read(fd, readBuf++, 1);  // blocking
      // ROS_INFO("After BUFFER READ");
      if (nr < 0)
      {
        ROS_ERROR("[arduino]: Read Error\n");
        reconnectUsb();
        return READ_ERROR;
      }
      else if (nr == 0)
      {
        ROS_ERROR("GOT TIMEOUT!");
        return READ_TIMEOUT;
      }
    // else if (nr != read_bytes)
    // {
    //   ROS_ERROR("[arduino]: Wrong number of bytes read, nr = %d, read = %d",nr,read_bytes);
    //   reconnectUsb();
    //   return INCORRECT_NUM_OF_BYTES;
    // }
    }
    // readBuf = buff;
    return NO_ERROR;
  }

  int ArduinoUsbInterface::sendServoCommand(
    const char& servoSelect, int value)
  {
    //int value is int because it is in the range of -100-100
    uint8_t bufOut;
    bufOut = servoValueToCommand(servoSelect, value);
    //value is mapped to an appropriate bufOut
    tcflush(fd, TCIOFLUSH);

    int nr = write(fd, &bufOut, 1);
    if (nr != 1)
    {
      ROS_ERROR("[arduino]: Write Error\n");
      reconnectUsb();
      return WRITE_ERROR;
    }
    return NO_ERROR;
  }

  int ArduinoUsbInterface::servoValueToCommand(
    const char& servoSelect, int value)
  {
    int positive = value + 100;
    switch(servoSelect)
    {
      case 'F':
        return (positive/200)*60;
        break;
      case 'R':
        return (positive/200)*60 + 60;
        break;
      default:
      break;
    }
  }

  int ArduinoUsbInterface::readBatteryValues(
    const char& batterySelect, uint16_t* value)
  {
    union
    {
      uint8_t batteryBufInUint8[BATTERY_NBYTES];
      uint16_t batteryBufInUint16;
    };

    int nr;
    uint8_t bufOut;

    switch (batterySelect)
    {
      case 'M':
        bufOut = COMMAND_BATTERY_MOTOR;
        break;
      case 'S':
        bufOut = COMMAND_BATTERY_ELECTRONICS;
        break;
      default:
        ROS_ERROR(
          "Undefined Battery selection. Motor battery selected by default");
        bufOut = COMMAND_BATTERY_ELECTRONICS;  // shouldn't get in there
        break;
    }

    int ret = readData(fd, bufOut, BATTERY_NBYTES, batteryBufInUint8);
    if ( ret==NO_ERROR )
      *value = batteryBufInUint16;
    return ret;
  }

  void ArduinoUsbInterface::reconnectUsb()
  {
    // reconnectUsb() should be called until communication is restored.
    close(fd);
    ROS_INFO("[arduino]: USB port closed\n");
    ros::Duration(1.5).sleep();

    openUsbPort();

    ROS_INFO("[arduino]: USB port reconnected successfully");
  }

}  // namespace arduino
}  // namespace pandora_hardware_interface
