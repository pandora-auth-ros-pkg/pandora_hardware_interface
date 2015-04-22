/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
* Author: Petros Evangelakos
*********************************************************************/

#include "linear_motor_com_interface/firgelli_com_interface.h"

namespace pandora_hardware_interface
{
namespace linear
{
  FirgelliComInterface::FirgelliComInterface()
  {
    mCtx_ = NULL;
    mHandle_ = NULL;
    mInterface_ = 0;
  libusb_init(&mCtx_);
  }

  FirgelliComInterface::~FirgelliComInterface()
  {
    int retval = libusb_release_interface(mHandle_, mInterface_);
    assert(retval == 0);
    if(mHandle_)
      libusb_close(mHandle_);
    libusb_exit(mCtx_);
  }

  void FirgelliComInterface::init()
  {
  }

  void FirgelliComInterface::openDevice()
  {
    //mHandle_ = libusb_open_device_with_vid_pid(mCtx_, vid, pid);
  }

  void FirgelliComInterface::closeDevice()
  {
  }

  bool FirgelliComInterface::write(const uint8_t* data, size_t size)
  {
  }

  bool FirgelliComInterface::read(uint8_t* data, size_t size)
  {
  }

  int FirgelliComInterface::readScaledFeedback()
  {
  }

  int FirgelliComInterface::setTarget(unsigned short target)
  {
  }
}  // namespace linear_motor
}  // namespace pandora_hardware_interface
