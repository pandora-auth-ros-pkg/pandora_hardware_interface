cmake_minimum_required(VERSION 2.8.3)
project(pandora_simulation_interface)
find_package(
  catkin
  REQUIRED COMPONENTS
  roscpp
  roslint
  gazebo_ros
  gazebo_plugins
  gazebo_ros_control
  pluginlib
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    pandora_simulation_interface
  CATKIN_DEPENDS
    roscpp
    roslint
    gazebo_ros
    gazebo_plugins
    gazebo_ros_control
    pluginlib
#  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS})

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS})

set(CMAKE_BUILD_TYPE Debug)

add_library(
  pandora_simulation_interface
  src/pandora_simulation_interface.cpp)
target_link_libraries(
  pandora_simulation_interface
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES})

# To check lint : catkin_make roslint_pandora_simulation_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/pandora_simulation_interface/pandora_simulation_interface.h
  src/pandora_simulation_interface.cpp

)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
