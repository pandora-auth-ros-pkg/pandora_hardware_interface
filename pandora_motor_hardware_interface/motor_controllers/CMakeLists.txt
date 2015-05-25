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
  skid_steer_new_controller
  src/skid_steer_new_controller.cpp)
target_link_libraries(
  skid_steer_new_controller
  ${catkin_LIBRARIES})