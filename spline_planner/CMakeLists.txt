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

# add_dependencies(spline_planner_2.py spline_planner_gencfg)

# catkin_python_setup()
catkin_package()

# catkin_install_python(PROGRAMS src/spline_planner.py
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

