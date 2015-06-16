cmake_minimum_required(VERSION 2.8.3)
project(pandora_arduino_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  controller_manager
  controller_interface
  sensor_msgs
  pandora_sensor_msgs
  pluginlib
)

set(THIS_PACKAGE_INCLUDE_DIRS
  arduino_controllers/include
  arduino_hardware_interface/include
  arduino_usb_interface/include
)

catkin_package(
  INCLUDE_DIRS ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES
    arduino_battery_controller  

  CATKIN_DEPENDS
    roscpp
    controller_manager
    controller_interface
    sensor_msgs
    pandora_sensor_msgs
    pluginlib
)

#~ set(CMAKE_BUILD_TYPE Debug)

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_subdirectory(arduino_controllers)
add_subdirectory(arduino_hardware_interface)
add_subdirectory(arduino_usb_interface)

# To check lint : catkin_make roslint_pandora_arduino_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -runtime/references")
  
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  arduino_controllers/*.h
  arduino_controllers/*.cpp
  arduino_hardware_interface/*.h
  arduino_hardware_interface/*.cpp
  arduino_usb_interface/*.h
  arduino_usb_interface/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
