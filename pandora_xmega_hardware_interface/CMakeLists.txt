cmake_minimum_required(VERSION 2.8.3)
project(pandora_xmega_hardware_interface)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  controller_manager
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    roslint
    controller_manager
#  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_BUILD_TYPE Debug)

add_executable(
  xmega_hardware_interface
  src/xmega_hardware_interface.cpp)
target_link_libraries(
  xmega_hardware_interface
  ${catkin_LIBRARIES})

# To check lint : catkin_make roslint_pandora_xmega_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/pandora_xmega_hardware_interface/power_supply.h
  include/pandora_xmega_hardware_interface/range_sensor.h
  include/pandora_xmega_hardware_interface/xmega_hardware_interface.h
  src/xmega_hardware_interface.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
