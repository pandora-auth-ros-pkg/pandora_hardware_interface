#include "epos_handler/abstract_epos_handler.h"

namespace pandora_hardware_interface {
namespace motor {

Current::Current() {}
Current::Current(const uint32_t left, const uint32_t right) {
  this->left = left;
  this->right = right;
}
Current::~Current() {}

Error::Error() {}
Error::~Error() {}
Error::Error(const uint32_t left, const uint32_t right) {
  this->left = left;
  this->right = right;
}

AbstractEposHandler::AbstractEposHandler() : gatewayImpl_(NULL)
{
}

uint32_t AbstractEposHandler::encodeToControlWord(const int& left, const int& right) {
  int signLeft = left < 0 ? 1 : 0;
  int signRight = right < 0 ? 1 : 0;

  int leftSpeedAbsolute = std::abs(left);
  int rightSpeedAbsolute = std::abs(right);


  uint32_t controlWord = 0;

  controlWord = (1 << 31) | (signRight << 30) | (rightSpeedAbsolute << 16) | (signLeft << 14) | (leftSpeedAbsolute);
  return controlWord;
}

AbstractEposHandler::~AbstractEposHandler()
{
}

}  // namespace motor
}  // namespace pandora_hardware_interface
