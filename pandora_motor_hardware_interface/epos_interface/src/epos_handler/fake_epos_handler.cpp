#include "epos_handler/fake_epos_handler.h"

namespace pandora_hardware_interface {
namespace motor {

FakeEposHandler::FakeEposHandler()
{
}

FakeEposHandler::~FakeEposHandler() {}

Kinematic::RPM FakeEposHandler::getRPM() {
  Kinematic::RPM retval;
  retval.setRPM(9999, 9999);
  return retval;
}

Current FakeEposHandler::getCurrent() {
  Current retval(1, 1);
  return retval;
}

Error FakeEposHandler::getError() {
  Error retval(0, 0);
  return retval;
}

epos::CommandStatus FakeEposHandler::writeRPM(const Kinematic::RPM& rpm) {
  return epos::SUCCESS;
}

}  // namespace motor
}  // namespace pandora_hardware_interface
