/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: George Kouros
*********************************************************************/

#include "leddar_usb_interface.h"


namespace pandora_hardware_interface
{
namespace leddar
{

  LeddarSerialInterface::LeddarSerialInterface(
    const std::string& device,
    int speed,
    int timeout)
  :
    device_(device),
    speed_(speed),
    timeout_(timeout)
  {
  }
  
  LeddarSerialInterface::~LeddarSerialInterface()
  {
    LeddarDisconnect(leddarHandle_);
    LeddarDestroy(leddarHandle_);
  }
  
  void LeddarSerialInterface::init()
  {
    leddarHandle_ = LeddarCreate();
    
    try
    {
      LeddarGetConnected(leddarHandle);
    }
    catch(serial::IOException& ex)
    {
      ROS_FATAL("[leddar] Error: Cannot open port !");
      ROS_FATAL("%s", ex.what());
      exit(-1);
    }
  }


  int LeddarSerialInterface::dataCallback()
  {
    // TODO
    
    return 0;
  }
  
  
  void LeddarSerialInterface::read()
  {
    int status = LeddarStartDataTransfer(leddarHandle_, LDDL_DETECTIONS);
    
    if ( status == LD_SUCCESS ) 
    {
      status = LeddarAddCallback(leddarHandle_, dataCallback, leddarHandle_);
    }

    LtChar lMessage[16];    

    if ( status != LD_SUCCESS )
    {
      LeddarGetErrorMessage(status, lMessage, ARRAY_LEN(lMessage));
      ROS_ERROR("[leddar] LeddarC Error(%d): %s\n", status, lMessage);
    }
    
    // TODO  check for current result and store it
    
    LeddarStopDataTransfer( leddarHandle_ );
    LeddarRemoveCallback( leddarHandle_, dataCallback, leddarHandle_ ); 
  }

} // namespace leddar
} // namespace pandora_hardware_interface
