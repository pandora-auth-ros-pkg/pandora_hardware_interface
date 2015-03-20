add_library(
  leddar_usb_interface
  src/leddar_usb_interface.cpp
  )

target_link_libraries(
  leddar_usb_interface 
  ${catkin_LIBRARIES}  
  )

add_executable(
  leddar_usb_interface_demo
  src/leddar_usb_interface_demo.cpp
  )

target_link_libraries(
  leddar_usb_interface_demo
  leddar_usb_interface
  libLeddarC.so  # shared libraries are located in /usr/lib
  ${catkin_LIBRARIES}
  )
