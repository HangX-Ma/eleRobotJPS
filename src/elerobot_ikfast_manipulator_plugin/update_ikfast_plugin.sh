search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=elerobot.srdf
robot_name_in_srdf=elerobot
moveit_config_pkg=elerobot_moveit_config
robot_name=elerobot
planning_group_name=manipulator
ikfast_plugin_pkg=elerobot_ikfast_manipulator_plugin
base_link_name=world
eef_link_name=elerobot_link7
ikfast_output_path=/home/contour/ws_catkin_elephant/src/elerobot_ikfast_manipulator_plugin/src/elerobot_manipulator_ikfast_solver.cpp

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
