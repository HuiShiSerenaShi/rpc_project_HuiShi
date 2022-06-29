cmake_minimum_required(VERSION 3.0.2)
project(edo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_commander
  moveit_msgs
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/main.py 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)