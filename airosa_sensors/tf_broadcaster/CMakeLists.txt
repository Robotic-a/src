cmake_minimum_required(VERSION 3.0.2)
project(tf_broadcaster)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)

## Catkin specific configuration
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  src/tf_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

