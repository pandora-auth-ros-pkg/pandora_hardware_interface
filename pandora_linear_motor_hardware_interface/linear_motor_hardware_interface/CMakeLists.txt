add_executable(
  linear_motor_hardware_interface_node
  src/linear_motor_hardware_interface.cpp
  src/linear_motor_hardware_interface_node.cpp)
target_link_libraries(
  linear_motor_hardware_interface_node
  ${catkin_LIBRARIES})
