cmake_minimum_required(VERSION 2.8.3)
project(imu_to_odom)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

catkin_package()

# Install Python nodes
catkin_install_python(PROGRAMS
  src/imu_to_odom_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

