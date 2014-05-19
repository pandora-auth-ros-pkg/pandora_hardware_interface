#include "epos_handler/serial_epos_handler.h"

namespace pandora_hardware_interface {
namespace motor {

SerialEposHandler::SerialEposHandler(const std::string& dev, const int& bauds, const int& time)
{
  gatewayImpl_.reset( new EposSerialGateway(dev, bauds, time) );
}

SerialEposHandler::~SerialEposHandler() {}

void SerialEposHandler::getRPM(int* left_back, int* left_front, int* right_back, int* right_front) {
  epos::Word out[2];
  int32_t rpmLeft_back, rpmRight_back,rpmLeft_front, rpmRight_front;
  gatewayImpl_->readObject(2, 0x2028, 0, &out[0]);
  rpmRight_back = (int16_t)out[1];
  gatewayImpl_->readObject(3, 0x2028, 0, &out[0]);
  rpmLeft_back = (int16_t)out[1];
  gatewayImpl_->readObject(4, 0x2028, 0, &out[0]);
  rpmLeft_front = (int16_t)out[1];
  gatewayImpl_->readObject(1, 0x206B, 0, &out[0]); // epos p
  rpmRight_front = (int32_t)out[1];
  if(rpmRight_front > 10000) {
    rpmRight_front -= 20000;
    ROS_DEBUG("WTF");
  }
 
  *left_back = rpmLeft_back;
  *left_front = rpmLeft_front;
  *right_back = -rpmRight_back;
  *right_front = -rpmRight_front;
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


