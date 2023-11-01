cmake_minimum_required(VERSION 3.0.2)
project(edo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_commander
  moveit_ros_planning_interface
  moveit_msgs
  rospy
  std_msgs
  gazebo_ros
  geometry_msgs
  roscpp
  moveit_core
  pluginlib
  tf2_kdl
  tf2_eigen
)

#moveit_build_options()

find_package(LAPACK REQUIRED)

find_package(gazebo REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}/..
  LIBRARIES gazebo_grasp_fix gazebo_grasp_msgs gazebo_version_helpers
  CATKIN_DEPENDS gazebo_ros geometry_msgs roscpp std_msgs 
  DEPENDS gazebo
)

###########
## Build ##
###########
# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)
include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_subdirectory(msgs)
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
)

## Declare a cpp library
add_library(gazebo_grasp_fix SHARED 
  src/gazebo_grasp_fix_plugin/GazeboGraspFix.cpp 
  src/gazebo_grasp_fix_plugin/GazeboGraspGripper.cpp 
  src/gazebo_grasp_fix_plugin/GazeboVersionHelpers.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(gazebo_grasp_plugin_node gazebo_grasp_plugin_generate_messages_cpp)
add_dependencies(gazebo_grasp_fix ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(gazebo_grasp_fix
   gazebo_grasp_msgs
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${Boost_LIBRARIES}
)

set(IKFAST_LIBRARY_NAME edo_edo_moveit_ikfast_plugin)
add_library(${IKFAST_LIBRARY_NAME} src/edo_edo_ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})
# suppress warnings about unused variables in OpenRave's solver code
target_compile_options(${IKFAST_LIBRARY_NAME} PRIVATE -Wno-unused-variable -Wno-unused-parameter)


#############
## Install ##
#############
install(TARGETS
  ${IKFAST_LIBRARY_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  FILES
  edo_edo_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
## Mark executables and/or libraries for installation
install(TARGETS gazebo_grasp_fix
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/gazebo_grasp_plugin/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
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