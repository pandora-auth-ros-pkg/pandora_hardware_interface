  #src/epos_gateway/epos_serial_gateway.cpp
#)
#target_link_libraries(epos_gateway ${catkin_LIBRARIES} )


#add_library(serial_epos_handler
  #src/epos_handler/abstract_epos_handler.cpp
  #src/epos_handler/serial_epos_handler.cpp
#)

#target_link_libraries(serial_epos_handler
  #epos_gateway
  #${catkin_LIBRARIES}
#)

message(STATUS ${CMAKE_SYSTEM_PROCESSOR})

# Download shared libraries
download_data(
  ${PROJECT_NAME}_download_libEposCmd5
  http://downloads.pandora.ee.auth.gr/data/shared_libraries/epos2/arm/hf/libEposCmd.so.5.0.1.0
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
  MD5 f73ea0c2c484c9150df3f55385ae18c1
  )

download_data(
  ${PROJECT_NAME}_download_libftd2xx1
  http://downloads.pandora.ee.auth.gr/data/shared_libraries/epos2/arm/hf/libftd2xx.so.1.2.8
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
  MD5 2db26cef49692cb7f0121f9202107dbf
  )

download_data(
  ${PROJECT_NAME}_download_libEposCmd
  http://downloads.pandora.ee.auth.gr/data/shared_libraries/epos2/arm/hf/libEposCmd.so
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
  MD5 f73ea0c2c484c9150df3f55385ae18c1
  )

download_data(
  ${PROJECT_NAME}_download_libftd2xx
  http://downloads.pandora.ee.auth.gr/data/shared_libraries/epos2/arm/hf/libftd2xx.so
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
  MD5 2db26cef49692cb7f0121f9202107dbf
  )

# Add devel/lib to link directories
link_directories(
    /usr/lib
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(Utils
  src/Utils/Utils.cpp
)

# Library for epos2 Gateway communication
add_library(epos2_gateway
  src/epos2_gateway/epos2_gateway.cpp
)

target_link_libraries(epos2_gateway
  EposCmd
  Utils
  ${catkin_LIBRARIES}
)

add_dependencies(epos2_gateway
  ${PROJECT_NAME}_download_libEposCmd
  ${PROJECT_NAME}_download_libftd2xx
  ${PROJECT_NAME}_download_libEposCmd5
  ${PROJECT_NAME}_download_libftd2xx1
)

add_library(serial_epos2_handler
  src/epos2_handler/serial_epos2_handler.cpp
  src/epos_handler/abstract_epos_handler.cpp
)
target_link_libraries(serial_epos2_handler
  epos2_gateway
  ${catkin_LIBRARIES}
)


#target_link_libraries(epos2_test
  #serial_epos2_handler
  #${catkin_LIBRARIES}
#)

#add_executable(epos_test
  #src/epos_handler/test.cpp
#)
#target_link_libraries(epos_test
  #serial_epos_handler
  #${catkin_LIBRARIES}
#)



