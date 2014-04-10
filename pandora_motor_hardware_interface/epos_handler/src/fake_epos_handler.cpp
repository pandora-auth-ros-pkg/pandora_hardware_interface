#include "fake_epos_handler.h"

FakeEposHandler::FakeEposHandler(): AbstractEposHandler()
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
