add_executable(pololu_jrk_communicator_node src/JrkCommunicator.cpp src/JrkSerial.cpp)

target_link_libraries(pololu_jrk_communicator_node ${catkin_LIBRARIES})
