#include "epos_handler/serial_epos_handler.h"

namespace pandora_hardware_interface {
namespace motor {

SerialEposHandler::SerialEposHandler(const std::string& dev, const int& bauds, const int& time)
{
  gatewayImpl_.reset( new EposSerialGateway(dev, bauds, time) );
}

SerialEposHandler::~SerialEposHandler() {}

void SerialEposHandler::getRPM(int* left, int* right) {
  epos::Word out[2];
  int32_t rpmLeft, rpmRight;
  gatewayImpl_->readObject(2, 0x2028, 0, &out[0]);
  rpmLeft = (int16_t)out[1];
  gatewayImpl_->readObject(2, 0x206B, 0, &out[0]);
  rpmRight = (int32_t)out[1];
  if(rpmRight > 10000) {
    rpmRight -= 20000;
    ROS_DEBUG("WTF");
  }
  rpmRight = -rpmRight;
  *left = rpmLeft;
  *right = rpmRight;
}

Current SerialEposHandler::getCurrent() {
  epos::Word out[2];
  Current cur;
  gatewayImpl_->readObject(2, 0x2027, 0, &out[0]);
  cur.left = (int16_t)out[1];

  gatewayImpl_->readObject(2, 0x2030, 0, &out[0]);
  cur.right = -(int16_t)out[1];
  return cur;
}

Error SerialEposHandler::getError() {
  epos::Word out[2];
  Error error;
  gatewayImpl_->readObject(2, 0x1003, 1, &out[0]);
  error.left = (int32_t)out[1];

  gatewayImpl_->readObject(2, 0x2081, 0, &out[0]);
  error.right = (int32_t)out[1];

  return error;
}

epos::CommandStatus SerialEposHandler::writeRPM(const int& left, const int& right) {
  ROS_INFO("setting speed %d, %d", left, right);
  //Right motor rpm speed needs to be reversed because of its placement in the vehicle
  uint32_t controlWord = encodeToControlWord(left, -right);
  epos::CommandStatus error = gatewayImpl_->writeObject(2, 0x200C, 1, controlWord);

  if(error != epos::SUCCESS) {
    ROS_ERROR("error setting speed");
  }
  return error;
}

}  // namespace motor
}  // namespace pandora_hardware_interface


