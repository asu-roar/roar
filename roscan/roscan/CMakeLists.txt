cmake_minimum_required(VERSION 3.0.2)
project(roscan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roar_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  roar_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/roscan_converter.py
  src/cansend.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
