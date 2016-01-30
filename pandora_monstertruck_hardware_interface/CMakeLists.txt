cmake_minimum_required(VERSION 2.8.3)
project(pandora_monstertruck_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roslint
  joint_limits_interface
  controller_manager
  controller_interface
  pandora_motor_hardware_interface
)

catkin_package(
  INCLUDE_DIRS
   include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    joint_limits_interface
    controller_manager
    controller_interface
    message_runtime
    pandora_motor_hardware_interface
  # DEPENDS
  # LIBRARIES
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(monstertruck_hardware_interface
  src/monstertruck_hardware_interface.cpp
)

target_link_libraries(monstertruck_hardware_interface
  serial_epos2_handler
  ${catkin_LIBRARIES}
)

add_executable(monstertruck_hardware_interface_node
  src/monstertruck_hardware_interface_node.cpp
)

target_link_libraries(monstertruck_hardware_interface_node
  monstertruck_hardware_interface
  ${catkin_LIBRARIES}
)

add_dependencies(monstertruck_hardware_interface_node
  ${catkin_EXPORTED_TARGETS}
)

# To check lint : catkin_make roslint_pandora_motor_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  *.h
  *.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
