cmake_minimum_required(VERSION 3.0.2)
project(airosa_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation message_runtime std_msgs geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  DynamixelState.msg
  SyncPosition.msg
  Imu.msg
  Joints.msg
  Pose.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
