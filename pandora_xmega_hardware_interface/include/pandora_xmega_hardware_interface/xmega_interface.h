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
#ifndef PANDORA_XMEGA_HARDWARE_INTERFACE_XMEGA_INTERFACE_H
#define PANDORA_XMEGA_HARDWARE_INTERFACE_XMEGA_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace pandora_xmega_hardware_interface
{
  class XmegaHandle
  {
  public:
    struct Data
    {
      Data()
      :
        name(),
        frame_id(),
        voltage(0),
        radiationType(0),
        fieldOfView(0),
        minRange(0),
        maxRange(0),
        range(0)
      {
      }

      std::string name;
      std::string frame_id;
      float* voltage;
      unsigned int* radiationType;
      float* fieldOfView;
      float* minRange;
      float* maxRange;
      float* range;
    };

    XmegaHandle(const Data& data = Data())
      : name_(data.name),
        frame_id_(data.frame_id),
        voltage_(data.voltage),
        radiationType_(data.radiationType),
        fieldOfView_(data.fieldOfView),
        minRange_(data.minRange),
        maxRange_(data.maxRange),
        range_(data.range)
    {
    }

    inline std::string getName() const
    {
      return name_;
    }
    inline std::string getFrameId() const
    {
      return frame_id_;
    }
    inline const float* getVoltage() const
    {
      return voltage_;
    }
    inline const unsigned int* getRadiationType() const
    {
      return radiationType_;
    }
    inline const float* getFieldOfView() const
    {
      return fieldOfView_;
    }
    inline const float* getMinRange() const
    {
      return minRange_;
    }
    inline const float* getMaxRange() const
    {
      return maxRange_;
    }
    inline const float* getRange() const
    {
      return range_;
    }

  private:
    std::string name_;
    std::string frame_id_;

    float* voltage_;
    unsigned int* radiationType_;
    float* fieldOfView_;
    float* minRange_;
    float* maxRange_;
    float* range_;
  };

  class XmegaInterface :
    public hardware_interface::HardwareResourceManager<XmegaHandle>
  {
  };
}  // namespace pandora_xmega_hardware_interface

#endif  // PANDORA_XMEGA_HARDWARE_INTERFACE_XMEGA_INTERFACE_H
