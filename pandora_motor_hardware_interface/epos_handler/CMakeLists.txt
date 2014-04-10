add_library(kinematic src/Kinematic.cpp)
target_link_libraries(kinematic ${catkin_LIBRARIES})

add_executable(epos
  src/abstractEposHandler.cpp
  src/eposHandler.cpp
  src/fakeEposHandler.cpp
  src/MainMotorController.cpp
  src/main.cpp
)
target_link_libraries(epos
  kinematic
  epos_gateway
  ${catkin_LIBRARIES}
)
