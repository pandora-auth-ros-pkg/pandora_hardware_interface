#ifndef ABSTRACTEPOSHANDLER_H
#define ABSTRACTEPOSHANDLER_H

#include "ros/ros.h"
#include "kinematic.h"
#include <epos_serial_gateway.h>
#include <stdint.h>

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
  ros::Time lastSetSpeedCall;
  bool falseSpeedCall;
  ros::Duration timeBetweenCalls;

 public:
  uint32_t encodeToControlWord(const Kinematic::RPM& rpm);
  virtual ~AbstractEposHandler();
  virtual Kinematic::RPM getRPM() = 0;
  virtual Current getCurrent() = 0;
  virtual Error getError() = 0;
  virtual epos::CommandStatus writeRPM(const Kinematic::RPM &rpm) = 0;
};


#endif // ABSTRACTEPOSHANDLER_H
