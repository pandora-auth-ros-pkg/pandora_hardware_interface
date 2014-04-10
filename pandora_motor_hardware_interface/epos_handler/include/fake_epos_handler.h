#ifndef FAKEEPOSHANDLER_H
#define FAKEEPOSHANDLER_H

#include "abstract_epos_handler.h"

class FakeEposHandler: public AbstractEposHandler {
  virtual Kinematic::RPM getRPM();
  virtual Current getCurrent();
  virtual Error getError();
  virtual epos::CommandStatus writeRPM(const Kinematic::RPM &rpm);
public:
  FakeEposHandler();
  virtual ~FakeEposHandler();
};

#endif // FAKEEPOSHANDLER_H
