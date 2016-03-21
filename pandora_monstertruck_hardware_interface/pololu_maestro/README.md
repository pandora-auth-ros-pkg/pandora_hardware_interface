# pololu_maestro

This package contains a driver for the **6-channel usb pololu maestro** servo controller.

In order to communicate with the device, a udev rule must be created by copying the **pololu_maestro.rules** file to **/etc/udev/rules.d**.  
This rule will allow
the user to communicate with the device and also force persistent naming of the
virtual ports, of the device (/dev/maestro_serial, /dev/maestro_ttl).

In addition the device must be configured (using the official maestro suite) as
shown below:
- serial mode: USB dual port
- Channels 0-3 Mode: servo
- Channels 4-5 Mode: input
- Servo Min Position: 1088
- Servo Max Position: 1840
- Servo Speed: 100

You can test the driver using the pololu_maestro_demo node by running:  
`rosrun pandora_monstertruck_hardware_interface pololu_maestro_demo <channel> <angle>`,
where channel denotes the servo channel of the device and should be in range [0,5],
while angle denotes the target position for the specified channel, in degrees, in
range [0,180].
