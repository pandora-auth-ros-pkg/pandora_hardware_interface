add_library(
  co2_sensor_controller
  src/co2_sensor_controller.cpp)
target_link_libraries(
  co2_sensor_controller
  ${catkin_LIBRARIES})

add_dependencies(
  co2_sensor_controller
  ${PROJECT_NAME}_gencpp)
  
add_library(
  thermal_sensor_controller
  src/thermal_sensor_controller.cpp)
target_link_libraries(
  thermal_sensor_controller
  ${catkin_LIBRARIES})
