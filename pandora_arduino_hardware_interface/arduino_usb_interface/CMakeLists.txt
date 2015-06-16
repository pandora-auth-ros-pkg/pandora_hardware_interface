add_library(
  arduino_usb_interface
  src/arduino_usb_interface.cpp)
target_link_libraries(
  arduino_usb_interface
  ${catkin_LIBRARIES})

add_executable(
  arduino_demo
  src/arduino_usb_interface_demo.cpp)
target_link_libraries(
  arduino_demo
  arduino_usb_interface
  ${catkin_LIBRARIES})
