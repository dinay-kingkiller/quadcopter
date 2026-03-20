cmake_minimum_required(VERSION 3.0.2)
project(quad_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Motor.msg
  Sensor.msg
  ICM20948.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime
)
