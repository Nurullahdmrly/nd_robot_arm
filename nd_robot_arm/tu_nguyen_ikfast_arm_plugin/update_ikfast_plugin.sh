search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=tu_nguyen.srdf
robot_name_in_srdf=tu_nguyen
moveit_config_pkg=nd_moveit_config
robot_name=tu_nguyen
planning_group_name=arm
ikfast_plugin_pkg=tu_nguyen_ikfast_arm_plugin
base_link_name=base_link
eef_link_name=link_4
ikfast_output_path=/home/oguz/ws_moveit/src/tu_nguyen_ikfast_arm_plugin/src/tu_nguyen_arm_ikfast_solver.cpp

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
