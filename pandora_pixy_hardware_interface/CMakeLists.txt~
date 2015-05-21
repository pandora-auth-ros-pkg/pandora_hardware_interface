cmake_minimum_required(VERSION 2.8.3)
project(pandora_pixy_hardware_interface)



set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/home/petros/pandora_ws/src/pandora_hardware_interface/pandora_pixy_hardware_interface/pixy_com_interface/cmake" )
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_PATH} )


find_package ( libusb-1.0 REQUIRED )
find_package ( OpenCV REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  controller_manager
  controller_interface
  image_transport
  message_generation
  roscpp
  tf
)

set(THIS_PACKAGE_INCLUDE_DIRS
    pixy_com_interface/include
    pixy_com_interface/include/common
    pixy_com_interface/src
    pixy_hardware_interface/include/pixy_hardware_interface
)


include_directories (${THIS_PACKAGE_INCLUDE_DIRS}
                     ${LIBUSB_1_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES pixyusb
  CATKIN_DEPENDS
    roscpp
    controller_manager
    std_msgs
    pandora_sensor_msgs
)



add_subdirectory(pixy_com_interface)
add_subdirectory(pixy_hardware_interface)


