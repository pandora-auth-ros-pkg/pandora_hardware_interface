add_library(epos_gateway
  src/epos_serial_gateway.cpp 
)
target_link_libraries(epos_gateway ${catkin_LIBRARIES} )



