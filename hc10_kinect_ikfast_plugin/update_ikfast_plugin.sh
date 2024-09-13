search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=motoman_hc10.srdf
robot_name_in_srdf=motoman_hc10
moveit_config_pkg=motoman_hc10_moveit_config
robot_name=motoman_hc10
planning_group_name=hc10_arm
ikfast_plugin_pkg=hc10_kinect_ikfast_plugin
base_link_name=base_link
eef_link_name=kinect_link
ikfast_output_path=/home/axel/ROS_ws/integration_ws/src/integration_ros_hc10_kinect/hc10_kinect_ikfast_plugin/src/hc10_kinect_ikfast_ikfast_solver.cpp
eef_direction="0 0 1"

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  --eef_direction $eef_direction\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
