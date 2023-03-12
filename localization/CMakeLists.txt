cmake_minimum_required(VERSION 3.0.2)
project(localization)

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

# catkin_install_python(PROGRAMS
#   src/script.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
