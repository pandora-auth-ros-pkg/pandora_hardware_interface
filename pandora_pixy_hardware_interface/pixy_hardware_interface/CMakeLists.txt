add_library(
  pixy_hardware_interface
  src/pixy_hardware_interface.cpp
  src/pixy_cam/pixycam.cpp
  src/pixy_cam/chirpmon.cpp
  src/pixy_cam/helpFunctions.cpp)
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
  ${catkin_LIBRARIES})

include_directories (include)
