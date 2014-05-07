cmake_minimum_required(VERSION 2.8.3)
project(pandora_linear_motor_hardware_interface)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  controller_manager
  controller_interface
  serial
)

set(THIS_PACKAGE_INCLUDE_DIRS
    jrk_interface/include
)

catkin_package(
  INCLUDE_DIRS ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    roscpp
    roslint
    controller_manager
    controller_interface
    serial
#  LIBRARIES
#  DEPENDS
)

#~ set(CMAKE_BUILD_TYPE Debug)

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(jrk_interface)
