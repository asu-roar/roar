cmake_minimum_required(VERSION 3.0.2)
project(leo_erc_teleop)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY 
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS 
    scripts/joy_pdu
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(
  PROGRAMS 
    scripts/key_teleop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)