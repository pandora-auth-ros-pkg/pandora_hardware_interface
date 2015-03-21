#link_directories(/usr/lib)

add_library(epos_gateway
  src/epos_gateway/epos_serial_gateway.cpp 
)
target_link_libraries(epos_gateway ${catkin_LIBRARIES} )


add_library(serial_epos_handler
  src/epos_handler/abstract_epos_handler.cpp
  src/epos_handler/serial_epos_handler.cpp
)

target_link_libraries(serial_epos_handler
  epos_gateway
  ${catkin_LIBRARIES}
)

#Library for epos2 Gateway communication
add_library(epos2_gateway
  src/epos2_gateway/epos2_gateway.cpp
)

target_link_libraries(epos2_gateway
  EposCmd
  ${catkin_LIBRARIES}
)


add_library(serial_epos2_handler
  src/epos2_handler/serial_epos2_handler.cpp
  src/epos_handler/abstract_epos_handler.cpp
)

target_link_libraries(serial_epos2_handler
  epos2_gateway
  ${catkin_LIBRARIES}
)

#add_executable(epos2_test
  #src/epos2_handler/epos2_test.cpp
#)

#target_link_libraries(epos2_test
  #serial_epos2_handler
  #${catkin_LIBRARIES}
#)

add_executable(epos_test
  src/epos_handler/test.cpp
)
target_link_libraries(epos_test
  serial_epos_handler
  ${catkin_LIBRARIES}
)



