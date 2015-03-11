#include "epos_handler/serial_epos2_handler.h"

int main(){
  pandora_hardware_interface::motor::SerialEpos2Handler s2("/dev/ttyS0", 9600, 1000);
}
