cmake_minimum_required(VERSION 3.0.2)
project(edo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_commander
  moveit_msgs
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install plugins
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Intall extra
install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/rqt_joint_trajectory_controller
  src/edo_move_group_interface.py
  src/main.py 
  src/edo_gripper_node.py
  src/edo_utils.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)