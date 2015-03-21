#include "epos_handler/serial_epos2_handler.h"

int main(){
  pandora_hardware_interface::motor::SerialEpos2Handler s2("/dev/ttyS0", 115200, 1000);
  s2.activate_profileVelocityMode(0);
  s2.moveWithVelocity(0,1000);
  while(true){}
  return 1;
}
