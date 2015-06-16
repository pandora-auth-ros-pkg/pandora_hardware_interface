add_executable(
  arduino_hardware_interface_node
  src/arduino_hardware_interface.cpp
  src/arduino_hardware_interface_node.cpp)

target_link_libraries(
  arduino_hardware_interface_node
  arduino_usb_interface
  ${catkin_LIBRARIES})
