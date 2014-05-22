#include "epos_handler/serial_epos_handler.h"

int main(int argc, char* argv[])
{
  pandora_hardware_interface::motor::SerialEposHandler
    motors("/dev/ttyUSB0", 115200, 500);
  
  if (argc != 3) {
    std::cerr << "Error: I need 2 speeds!!" << std::endl;
    exit(-1);
  }

  int speedLeft = atoi(argv[1]);
  int speedRight = atoi(argv[2]);

  motors.writeRPM(speedLeft, speedRight);

  sleep(1);
  return 0;
}
