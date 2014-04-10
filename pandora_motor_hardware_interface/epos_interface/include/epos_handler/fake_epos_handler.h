#ifndef FAKE_EPOS_HANDLER_H
#define FAKE_EPOS_HANDLER_H

#include "epos_handler/abstract_epos_handler.h"

namespace pandora_hardware_interface {
namespace motor {

class FakeEposHandler : public AbstractEposHandler {

 public:
  FakeEposHandler();
  virtual ~FakeEposHandler();

  virtual Kinematic::RPM getRPM();
  virtual Current getCurrent();
  virtual Error getError();
  virtual epos::CommandStatus writeRPM(const Kinematic::RPM &rpm);
};

}  // namespace motor
}  // namespace pandora_hardware_interface

#endif // FAKE_EPOS_HANDLER_H
