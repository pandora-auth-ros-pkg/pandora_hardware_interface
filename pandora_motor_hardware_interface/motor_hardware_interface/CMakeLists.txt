link_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(four_wheel_steer_hardware_interface
  src/four_wheel_steer_hardware_interface.cpp
)

target_link_libraries(four_wheel_steer_hardware_interface
  serial_epos_handler
  ${catkin_LIBRARIES}
)

add_library(motor_hardware_interface
  src/motor_hardware_interface.cpp
)

target_link_libraries(motor_hardware_interface
  serial_epos2_handler
  ${catkin_LIBRARIES}
)

add_executable(motor_hardware_interface_node
  src/motor_hardware_interface_node.cpp
)

target_link_libraries(motor_hardware_interface_node
  motor_hardware_interface
  ${catkin_LIBRARIES}
)

add_dependencies(
  motor_hardware_interface_node
  ${catkin_EXPORTED_TARGETS}
)
