search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=edo.srdf
robot_name_in_srdf=edo
moveit_config_pkg=edo
robot_name=edo
planning_group_name=edo
ikfast_plugin_pkg=edo_edo_ikfast_plugin
base_link_name=edo_base_link
eef_link_name=edo_link_ee
ikfast_output_path=/home/lorenzobusellato/univr/Robot_programming_and_control/catkin_ws/src/rpc_project/edo_edo_ikfast_plugin/src/edo_edo_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
