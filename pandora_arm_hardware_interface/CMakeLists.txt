cmake_minimum_required(VERSION 2.8.3)
project(pandora_arm_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Co2Msg.msg
  )

generate_messages(
  DEPENDENCIES
    std_msgs
)


catkin_package(
  CATKIN_DEPENDS
    message_generation
    message_runtime
    std_msgs
)