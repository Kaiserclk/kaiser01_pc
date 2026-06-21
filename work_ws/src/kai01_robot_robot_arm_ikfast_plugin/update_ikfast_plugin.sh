search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=kai01_robot.srdf
robot_name_in_srdf=kai01_robot
moveit_config_pkg=kai01_robot_moveit_config
robot_name=kai01_robot
planning_group_name=robot_arm
ikfast_plugin_pkg=kai01_robot_robot_arm_ikfast_plugin
base_link_name=base_link
eef_link_name=servo_6
ikfast_output_path=/home/kaiser/ros2-humble/temp_ws/kai01_robot_robot_arm_ikfast_plugin/src/kai01_robot_robot_arm_ikfast_solver.cpp

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
