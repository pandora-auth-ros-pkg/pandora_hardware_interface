add_library(epos_gateway
  src/epos_gateway/epos_serial_gateway.cpp 
)
target_link_libraries(epos_gateway ${catkin_LIBRARIES} )

add_library(kinematic src/epos_handler/kinematic.cpp)
target_link_libraries(kinematic ${catkin_LIBRARIES})

#~ add_executable(epos
  #~ src/epos_handler/abstract_epos_handler.cpp
  #~ src/epos_handler/serial_epos_handler.cpp
  #~ src/epos_handler/fake_epos_handler.cpp
  #~ src/epos_handler/main_motor_controller.cpp
  #~ src/epos_handler/main.cpp
#~ )
#~ target_link_libraries(epos
  #~ kinematic
  #~ epos_gateway
  #~ ${catkin_LIBRARIES}
#~ )

add_library(epos
  src/epos_handler/abstract_epos_handler.cpp
  src/epos_handler/serial_epos_handler.cpp
  src/epos_handler/main_motor_controller.cpp
)
target_link_libraries(epos
  epos_gateway
  ${catkin_LIBRARIES}
)


