add_library(
  skid_steer_drive_controller
  src/skid_steer_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp)
target_link_libraries(
  skid_steer_drive_controller
  ${catkin_LIBRARIES})

#Add new controller as library
add_library(
  skid_steer_test_controller
  src/skid_steer_new_controller.cpp)
target_link_libraries(
  skid_steer_test_controller
  ${catkin_LIBRARIES})

#Add torque controller as library
add_library(
  skid_steer_torque_controller
  src/skid_steer_torque_controller.cpp)
add_dependencies(skid_steer_torque_controller pandora_sensor_msgs)  #Check Again!!!
target_link_libraries(
  skid_steer_torque_controller
  ${catkin_LIBRARIES})