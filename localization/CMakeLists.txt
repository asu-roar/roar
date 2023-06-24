cmake_minimum_required(VERSION 3.0.2)
project(localization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roar_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  roar_msgs
  tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(localization ukf.cpp)

target_link_libraries(localization
   ${catkin_LIBRARIES}
 )

 link_directories(/usr/local/include)

# catkin_install_python(PROGRAMS
#   src/script.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
