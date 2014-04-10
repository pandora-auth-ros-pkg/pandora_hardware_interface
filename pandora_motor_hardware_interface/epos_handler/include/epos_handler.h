#ifndef eposHandler_H
#define eposHandler_H

#endif // eposHandler_H


#include "abstract_epos_handler.h"


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
