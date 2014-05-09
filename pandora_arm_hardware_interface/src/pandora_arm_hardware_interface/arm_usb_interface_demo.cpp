#include "pandora_arm_hardware_interface/arm_usb_interface.h"

int main(int argc, char** argv)
{
  ros::Time::init();

  pandora_hardware_interface::arm::ArmUSBInterface arm;

  uint8_t temperatureCenter[64];
  uint8_t temperatureLeft[64];
  uint8_t temperatureRight[64];
//  uint8_t temperature[64];

  while(1) {

    arm.grideyeValuesGet('C', temperatureCenter);

    arm.grideyeValuesGet('L', temperatureLeft);

    arm.grideyeValuesGet('R', temperatureRight);

    arm.co2ValueGet();

    ros::Duration(0.02).sleep();
  }

  return 0;
}

