# Drone Race project

## Prerequisites
- ROS Kinetic
- ETH RotorS
- ethzasl_msf
- rotors_simulator_demos

See more detailed installation instructions in
[install.md](https://github.com/Veilkrand/drone_race/blob/master/install.md).

## Run environment

1. Launch Gazebo with test track, and RotorS plugins:
  * With vi camera`$ roslaunch gazebo_assets_drone_race test_rotors_msf_camera.launch`
  * Or without camera (faster): `$ roslaunch gazebo_assets_drone_race rotors_msf_test_track.launch`
2. Run spline planner to plan a path for the drone:
  * `$ rosrun spline_planner spline_planner.py`
3. Run waypoint controller to control the drone:
  * `$ rosrun waypoint_controller control_node.py _target_exit_speed:=1`
4. Run ground truth and visualizations:
  * `$ rosrun drone_map_builder gt_gates_publisher_node.py`
  * `$ rosrun drone_map_builder gt_ego_position.py`
5. Run RVIZ
  * `$ rviz -d $(rospack find drone_map_builder)/configs/default.rviz`

## RotorS commands examples

`$ rosrun rotors_gazebo waypoint_publisher 0 0 1 180 0 __ns:=firefly`

