 add_library(
  arduino_battery_controller
  src/battery_controller.cpp)
add_dependencies(
  arduino_battery_controller
  ${PROJECT_NAME}_gencpp
  pandora_sensor_msgs_gencpp)
target_link_libraries(
  arduino_battery_controller
  ${catkin_LIBRARIES})


