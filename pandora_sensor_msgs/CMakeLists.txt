cmake_minimum_required(VERSION 2.8.3)
project(pandora_sensor_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  )


add_message_files(
  FILES
    BatteryMsg.msg
    Co2Msg.msg
    ImuRPY.msg
    MotorCurrents.msg
    Temperature.msg
    ThermalMeanMsg.msg
    TorqueMsg.msg
    )

generate_messages(
  DEPENDENCIES
    std_msgs
  )
  

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
  )

