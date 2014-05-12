add_library(
  co2_sensor_controller
  src/co2_sensor_controller.cpp)
target_link_libraries(
  co2_sensor_controller
  ${catkin_LIBRARIES})
