cmake_minimum_required(VERSION 2.8.3)
project(pandora_monstertruck_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  std_msgs
  geometry_msgs
  ackermann_msgs
  controller_manager
  controller_interface
  dual_controller_interface
  joint_limits_interface
  pandora_motor_hardware_interface
  pluginlib
)

set(THIS_PACKAGE_INCLUDE_DIRS
  monstertruck_steer_drive_controller/include
  monstertruck_hardware_interface/include
  pololu_maestro_driver/include
)

catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    ackermann_msgs
    controller_manager
    controller_interface
    dual_controller_interface
    joint_limits_interface
    pandora_motor_hardware_interface
    pluginlib
  DEPENDS
  LIBRARIES
    monstertruck_steer_drive_controller
)


include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(monstertruck_hardware_interface)
add_subdirectory(monstertruck_steer_drive_controller)
add_subdirectory(pololu_maestro_driver)


# To check lint : catkin_make roslint_pandora_motor_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  *.h
  *.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
