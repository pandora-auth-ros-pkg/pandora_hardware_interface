link_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(monstertruck_hardware_interface
  src/monstertruck_hardware_interface.cpp
)

target_link_libraries(monstertruck_hardware_interface
  serial_epos2_handler
  pololu_maestro
  ${catkin_LIBRARIES}
)

add_executable(monstertruck_hardware_interface_node
  src/monstertruck_hardware_interface_node.cpp
)

target_link_libraries(monstertruck_hardware_interface_node
  monstertruck_hardware_interface
  ${catkin_LIBRARIES}
)

add_dependencies(monstertruck_hardware_interface_node
  ${catkin_EXPORTED_TARGETS}
)
