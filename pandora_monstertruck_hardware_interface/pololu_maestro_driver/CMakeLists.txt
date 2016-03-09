add_library(pololu_maestro_driver
  src/pololu_maestro_driver.cpp
)

target_link_libraries(pololu_maestro_driver
  ${CATKIN_LIBRARIES}
)

add_executable(pololu_maestro_test
  src/pololu_maestro_test.cpp
)

target_link_libraries(pololu_maestro_test
  pololu_maestro_driver
  ${catkin_LIBRARIES}
)

