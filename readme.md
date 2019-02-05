# Drone Race project

## Prerequisites
- ROS Kinetic
- ETH RotorS
- ethzasl_msf
- rotors_simulator_demos


## Run environment

1. Launch Gazebo with test track, and RotorS plugins:
With vi camera`$ roslaunch gazebo_assets_drone_race test_rotors_msf_camera.launch`
Or without camera (faster): `$ roslaunch gazebo_assets_drone_race rotors_msf_test_track.launch`
2. Run waypoint controller to control the drone:
`$ rosrun waypoint_controller control_node.py _target_exit_speed:=1`
3. Run ground truth and visualizations:
`$ rosrun drone_map_builder gt_gates_publisher_node.py`
`$ rosrun drone_map_builder gt_ego_position.py`
4. Run RVIZ
`$ rviz -d $(rospack find drone_map_builder)/configs/default.rviz`
