add_library(pololu_maestro
  src/pololu_maestro.cpp
)

target_link_libraries(pololu_maestro
  ${CATKIN_LIBRARIES}
)

add_executable(pololu_maestro_demo
  src/pololu_maestro_demo.cpp
)

target_link_libraries(pololu_maestro_demo
  pololu_maestro
  ${catkin_LIBRARIES}
)

