#ifndef EPOS_HANDLER_H
#define EPOS_HANDLER_H

#include "abstract_epos_handler.h"

namespace pandora_hardware_interface {
namespace motor {

class EposHandler: public AbstractEposHandler {
  EposSerialGateway gateway;

public:
  EposHandler(const std::string& dev, const int& bauds, const int& time);
  virtual ~EposHandler();
  virtual Kinematic::RPM getRPM();
  virtual Current getCurrent();
  virtual Error getError();
  virtual epos::CommandStatus writeRPM(const Kinematic::RPM &rpm);
};

}  // namespace motor
}  // namespace pandora_hardware_interface

#endif  // EPOS_HANDLER_H
