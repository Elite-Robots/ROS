search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=ec66.srdf
robot_name_in_srdf=ec66
moveit_config_pkg=ec66_moveit_config
robot_name=ec66
planning_group_name=manipulator
ikfast_plugin_pkg=elite_moveit_ikfast_plugin_ec66
base_link_name=world
eef_link_name=flan
ikfast_output_path=/mnt/d/Code/GitHub/elite_robot_ros/src/elite_moveit_ikfast_plugins/elite_moveit_ikfast_plugin_ec66/src/ec66_manipulator_ikfast_solver.cpp

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
