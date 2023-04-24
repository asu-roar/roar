cmake_minimum_required(VERSION 3.0.2)
project(supervisor)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roar_msgs
)


catkin_python_setup()


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
   scripts/autonomous_supervisor.py
   scripts/roar_supervisor.py
   test/manual.py
   test/autonomous.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )