cmake_minimum_required(VERSION 2.8.3)
project(pandora_arm_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roslint
  controller_manager
  controller_interface
  sensor_msgs
  pluginlib
)

set(THIS_PACKAGE_INCLUDE_DIRS
    arm_controllers/include
    arm_hardware_interface/include
)

add_message_files(
  FILES
  Co2Msg.msg
)

generate_messages(
  DEPENDENCIES
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    message_generation
    message_runtime
    std_msgs
)

#~ set(CMAKE_BUILD_TYPE Debug)

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(arm_controllers)
add_subdirectory(arm_hardware_interface)

# To check lint : catkin_make roslint_pandora_arm_hardware_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")
  
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  arm_controllers/*.h 
  arm_controllers/*.cpp
  arm_hardware_interface/*.h 
  arm_hardware_interface/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
