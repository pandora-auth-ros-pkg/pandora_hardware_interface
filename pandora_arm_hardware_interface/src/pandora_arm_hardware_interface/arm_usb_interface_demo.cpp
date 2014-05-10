#include "pandora_arm_hardware_interface/arm_usb_interface.h"

int main(int argc, char** argv)
{
  ros::Time::init();

  pandora_hardware_interface::arm::ArmUSBInterface arm;

  uint8_t temperature[64];

  while (1)
  {

    arm.grideyeValuesGet('C', temperature);

    arm.grideyeValuesGet('L', temperature);

    arm.grideyeValuesGet('R', temperature);

    arm.co2ValueGet();

    ros::Duration(0.1).sleep();
  }

  return 0;
}

