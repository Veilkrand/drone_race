# Waypoint controller

## Run

Launch msf controller with race track and waypoint controller:
```
roslaunch gazebo_assets_drone_race test_msf.launch
rosrun waypoint_controller control_node.py
rosrun rotors_gazebo waypoint_publisher 0 0 1 0 0 __ns:=firefly
```