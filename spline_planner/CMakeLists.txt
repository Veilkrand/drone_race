cmake_minimum_required(VERSION 2.8.3)
project(spline_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
  dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/Planner.cfg
)

# catkin_python_setup()
catkin_package()

# catkin_install_python(PROGRAMS scripts/planner.py
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

