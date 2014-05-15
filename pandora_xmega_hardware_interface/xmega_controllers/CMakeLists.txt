add_library(
  power_supply_controller
  src/power_supply_controller.cpp)
add_dependencies(
  power_supply_controller
  ${PROJECT_NAME}_gencpp)
target_link_libraries(
  power_supply_controller
  ${catkin_LIBRARIES})

add_library(
  range_sensor_controller
  src/range_sensor_controller.cpp)
target_link_libraries(
  range_sensor_controller
  ${catkin_LIBRARIES})
