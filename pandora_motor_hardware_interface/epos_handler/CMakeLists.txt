add_library(kinematic src/kinematic.cpp)
target_link_libraries(kinematic ${catkin_LIBRARIES})

add_executable(epos
  src/abstract_epos_handler.cpp
  src/epos_handler.cpp
  src/fake_epos_handler.cpp
  src/main_motor_controller.cpp
  src/main.cpp
)
target_link_libraries(epos
  kinematic
  epos_gateway
  ${catkin_LIBRARIES}
)
