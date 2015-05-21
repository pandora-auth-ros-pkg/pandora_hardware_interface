add_library(
  pixy_hardware_interface
  src/pixy_hardware_interface.cpp)
target_link_libraries(pixy_hardware_interface
  ${OpenCV_LIBRARIES}
)

add_executable(
  pixy_hardware_interface_node
  src/pixy_hardware_interface_node.cpp)
target_link_libraries(
  pixy_hardware_interface_node
  pixy_hardware_interface
  pixyusb
  ${catkin_LIBRARIES}
  ${LIBUSB_1_LIBRARY}
  ${Boost_LIBRARIES}
)

include_directories (include)
