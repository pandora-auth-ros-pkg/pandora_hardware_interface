#ifndef ABSTRACTEPOSHANDLER_H
#define ABSTRACTEPOSHANDLER_H

#include "ros/ros.h"
#include <boost/scoped_ptr.hpp>
#include "epos_handler/kinematic.h"
#include <epos_gateway/epos_serial_gateway.h>
#include <stdint.h>

namespace pandora_hardware_interface {
  namespace motor {

  class Error {
   public:
    uint32_t left;
    uint32_t right;
    Error(const uint32_t left, const uint32_t right);
    Error();
    ~Error();
  };
  
  class Current {
   public:
    uint32_t left;
    uint32_t right;
    Current(const uint32_t left, const uint32_t right);
    Current();
    ~Current();
  };
  
  class AbstractEposHandler
  {
   protected:
    boost::scoped_ptr<AbstractEposGateway> gatewayImpl_;

   public:
    AbstractEposHandler();
    virtual ~AbstractEposHandler();
    uint32_t encodeToControlWord(const Kinematic::RPM& rpm);
    virtual Kinematic::RPM getRPM() = 0;
    virtual Current getCurrent() = 0;
    virtual Error getError() = 0;
    virtual epos::CommandStatus writeRPM(const Kinematic::RPM &rpm) = 0;
  };

  }  // namespace motor
}  // namespace pandora_hardware_interface

#endif // ABSTRACTEPOSHANDLER_H
