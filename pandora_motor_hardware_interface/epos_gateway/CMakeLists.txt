add_library(epos_gateway
  src/Rs232.cpp
  src/EposRs232Gateway.cpp 
)
target_link_libraries(epos_gateway ${catkin_LIBRARIES} )



