cmake_minimum_required(VERSION 3.0.2)
project(airosa_controller)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  airosa_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs airosa_msgs
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts (ensure they are executable and have shebang)
catkin_install_python(PROGRAMS
  src/controller.py
  src/tf_broadcaster.py
  src/operator.py
  src/sync_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
