/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, P.A.N.D.O.R.A. Team.
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
* Author: George Kouros
*********************************************************************/

#include "pololu_maestro_driver/pololu_maestro_driver.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trax_ahrs_configuration_node");
  ros::NodeHandle nodeHandle;

  pandora_hardware_interface::pololu_maestro::PololuMaestro
    maestro("/dev/ttyACM0", 9600, 500);

  if (argc == 3)
  {
    unsigned char channel = argv[1][0]-48;

    std::istringstream iss(argv[2]);
    int command;
    iss >> command;

    ROS_INFO("Argumentss: channel=%i, command=%d [degrees]", channel, command);

    if (argc == 3)
    {
      maestro.setTarget(channel, static_cast<double>(command)/180.0*M_PI);
      ROS_INFO("Feedback: %f", maestro.readPosition(0.0));
      ROS_INFO("AnalogVoltage on channel 5: %f", maestro.readVoltage(5));
    }
  }
  else
  {
    ROS_INFO("Run the command via command line giving two arguments "
     "channel[0-5] and command in degrees");
  }
}
