cmake_minimum_required(VERSION 3.0.2)
project(airosa_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY meshes launch urdf rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

