cmake_minimum_required(VERSION 2.8.3)
project(pandora_leddar_hardware_interface)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


add_message_files(
  FILES
  Message1.msg
  Message2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pandora_leddar_hardware_interface
  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS system_lib
)

include_directories(include)

add_library(pandora_leddar_hardware_interface
  src/${PROJECT_NAME}/pandora_leddar_hardware_interface.cpp
)

add_executable()

add_dependencies(pandora_leddar_hardware_interface_node pandora_leddar_hardware_interface_generate_messages_cpp)
target_link_libraries(pandora_leddar_hardware_interface_node
  ${catkin_LIBRARIES}
)

install(PROGRAMS
  scripts/my_python_script
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

