cmake_minimum_required(VERSION 3.0.2)
project(teleoperation)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roar_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES teleoperation
#  CATKIN_DEPENDS message_generation message_runtime roar_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)