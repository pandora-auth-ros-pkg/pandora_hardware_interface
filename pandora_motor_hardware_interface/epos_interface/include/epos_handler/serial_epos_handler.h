#ifndef SERIAL_EPOS_HANDLER_H
#define SERIAL_EPOS_HANDLER_H

#include "epos_handler/abstract_epos_handler.h"

namespace pandora_hardware_interface {
namespace motor {

class SerialEposHandler: public AbstractEposHandler {
 public:
  SerialEposHandler(const std::string& dev, const int& bauds, const int& time);
  virtual ~SerialEposHandler();
  virtual void getRPM(int* left_back, int* left_front, int* right_back, int* right_front);
  virtual Current getCurrent();
  virtual Error getError();
  virtual epos::CommandStatus writeRPM(const int& left_rpm, const int& right_rpm);
};

}  // namespace motor
}  // namespace pandora_hardware_interface

#endif  // SERIAL_EPOS_HANDLER_H
