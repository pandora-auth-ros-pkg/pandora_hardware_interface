#include "pandora_xmega_hardware_interface/xmega_serial_interface.h"

int main(int argc, char** argv)
{
  pandora_xmega::XmegaSerialInterface xmega("/dev/ttyUSB0", 115200, 100);
  xmega.init();
  
  
  float psu = 0, motor = 0;
  while(1) {
    
    xmega.read();
    xmega.getBatteryData(&psu, &motor);
    
    std::cout << psu << " " << motor << std::endl;
  }
  return 0;
}
