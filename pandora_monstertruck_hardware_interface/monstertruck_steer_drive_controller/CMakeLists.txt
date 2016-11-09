add_library(monstertruck_steer_drive_controller
  src/odometry.cpp
  src/monstertruck_steer_drive_controller.cpp
)

add_dependencies(monstertruck_steer_drive_controller
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  monstertruck_steer_drive_controller
  ${catkin_LIBRARIES}
)
