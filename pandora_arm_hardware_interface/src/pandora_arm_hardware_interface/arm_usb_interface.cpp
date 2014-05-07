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

#include "pandora_arm_hardware_interface/arm_usb_interface.h"

using namespace std;

int main(void)
{
  int fd;

//	fd = open("/dev/head", O_RDWR | O_NOCTTY | O_NDELAY);	//to make read non-blocking
  fd = open("/dev/head", O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    ROS_ERROR("[Head]: cannot open usb port\n");
    ROS_ERROR("[Head]: open() failed with error [%s]\n", strerror(errno));
    return -1;
  }
  else
  {
    ROS_INFO("[Head]: usb port successfully opened\n");
  }
  ros::Duration(0.03).sleep(); //needs some time to initialize, even though it opens succesfully. tcflush() didn't work
  //without waiting at least 8 ms

  union
  {
    unsigned char CO2bufIN[CO2_NBYTES];
    float CO2bufIN_float;
  };

  unsigned char GEYEbufIN[GEYE_NBYTES];

  int bufOUT;

  int nr;

//	fcntl(fd, F_SETFL, FNDELAY);	//make read() non-blocking
//	fcntl(fd, F_SETFL, 0);	//make read() blocking

  for (;;)
  {
    tcflush(fd, TCIFLUSH); //empties incoming buffer

    bufOUT = COMMAND_GEYE_CENTER;
    nr = write(fd, (const void *)&bufOUT, COMMAND_NBYTES);
    if (nr != 1)
    {
      reconnectUSB(fd);
      continue;
    }
    nr = read(fd, GEYEbufIN, GEYE_NBYTES); //blocking
    if (nr < 0) {
      cout << "Read Error" << endl;
    } else {
      cout << "GEYE_CENTER = ";
      for (int i = 0; i < GEYE_NBYTES; ++i)
      {
        cout << (int)GEYEbufIN[i] << " ";
      }
      cout << endl;
    }

    bufOUT = COMMAND_GEYE_LEFT;
    nr = write(fd, (const void *)&bufOUT, COMMAND_NBYTES);
    if (nr != 1)
    {
      reconnectUSB(fd);
      continue;
    }
    nr = read(fd, GEYEbufIN, GEYE_NBYTES); //blocking
    if (nr < 0) {
      cout << "Read Error" << endl;
    } else {
    cout << "GEYE_LEFT = ";
      for (int i = 0; i < GEYE_NBYTES; ++i)
      {
        cout << (int)GEYEbufIN[i] << " ";
      }
      cout << endl;
    }

    bufOUT = COMMAND_GEYE_RIGHT;
    nr = write(fd, (const void *)&bufOUT, COMMAND_NBYTES);
    if (nr != 1)
    {
      reconnectUSB(fd);
      continue;
    }
    nr = read(fd, GEYEbufIN, GEYE_NBYTES); //blocking
    if (nr < 0) {
      cout << "Read Error" << endl;
    } else {
    cout << "GEYE_RIGHT = ";
      for (int i = 0; i < GEYE_NBYTES; ++i)
      {
        cout << (int)GEYEbufIN[i] << " ";
      }
      cout << endl;
    }

    bufOUT = COMMAND_CO2;
    nr = write(fd, (const void *)&bufOUT, COMMAND_NBYTES);
    if (nr != 1)
    {
      reconnectUSB(fd);
      continue;
    }
    nr = read(fd, CO2bufIN, CO2_NBYTES); //blocking
    if (nr < 0) {
      cout << "Read Error" << endl;
    } else {
      cout << "CO2 = " << CO2bufIN_float << endl;
    }

    ros::Duration(0.02).sleep();
  }

  close(fd);
  ROS_INFO("[Head]: usb port closed because of program termination\n");
  return EXIT_SUCCESS;
}

int reconnectUSB(int fd)
{
  ROS_ERROR("[Head]: Write Error\n");
  close(fd);
  ROS_INFO("[Head]: usb port closed\n");
  //If usb disconnects and reconnects again 1.5s should be fine, if uC resets 4.5s required.
  //reconnectUSB() is called until communication is restored.
  ros::Duration(1.5).sleep();
  fd = open("/dev/head", O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    ROS_FATAL("[Head]: cannot reopen usb port\n");
    ROS_FATAL("[Head]: open() failed with error [%s]\n", strerror(errno));
    return -1;
  }
  else
  {
    ROS_INFO("[Head]: usb port successfully reopened\n");
  }
  ros::Duration(0.03).sleep();
}

