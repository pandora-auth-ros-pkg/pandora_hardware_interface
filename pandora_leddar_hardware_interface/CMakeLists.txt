cmake_minimum_required(VERSION 2.8.3)
project(pandora_leddar_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_runtime
  roslint
  )

set(THIS_PACKAGE_INCLUDE_DIRS
  leddar_serial_interface/include
  )

catkin_package(
  INCLUDE_DIRS 
    ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES 
  CATKIN_DEPENDS
    roscpp
    roslint
  )

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

add_subdirectory(leddar_serial_interface)

# To check lint : catkin_make roslint_pandora_arm_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -runtime/references")
  FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  leddar_serial_interface/leddar_serial_interface.h
  leddar_serial_interface/leddar_serial_interface.cpp
  leddar_serial_interface/leddar_serial_interface_demo.cpp
  )

roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
