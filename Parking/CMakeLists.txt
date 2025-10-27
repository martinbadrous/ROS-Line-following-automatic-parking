cmake_minimum_required(VERSION 3.0.2)
project(Parking)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/auto_parking.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(${catkin_INCLUDE_DIRS})
