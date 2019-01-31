# Drone Race project

## Prerequisites
- ROS Kinetic
- ETH RotorS
- ethzasl_msf
- rotors_simulator_demos


## Run environment

1. Launch Gazebo with test track, and RotorS plugins:
`$ roslaunch gazebo_assets_drone_race rotors_msf_test_track.launch`
2. Run waypoint controller to control the drone:
`$ rosrun waypoint_controller control_node.py`
3. Run ground truth and visualizations:
`$ rosrun drone_map_builder gt_gates_publisher_node.py`
`$ rosrun drone_map_builder observed_gates_map_node.py`
4. Run RVIZ
`$ rviz -d $(rospack find drone_map_builder)/configs/default.rviz`
