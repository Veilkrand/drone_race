# ROS Gazebo Assets for race drone simulation
> Alberto S. Naranjo Galet. Dec. 2018.
> This project is licensed under the terms of the MIT license.

![Race Track](https://github.com/Veilkrand/gazebo_assets_drone_race/blob/master/images/default_gzclient_camera(1)-2018-12-05T13_20_12.687049.jpg)

Some assets to start simulating a simple drone race track in Gazebo. Markers are made using Aruco markers with a 4x4 dictionary. Dimensions are 0.5mx0.5m
To finish one lap you need to navigate the closest gate (from the front of the drone) and then land in the landing spot. For more than one lap the landing spot is a regular gate you need to pass 1 meter above.
Gate are solid and you can't cross them, use a point in space 1 meter above the center of the marker with the same pose.

## Instalation
Deploy in your Catkin workspace `/src` and build the packages `catkin build`. Source the workspace `$ . /devel/setup.bash`. Package name is `gazebo_assets_drone_race`.

## Launch world
Run `$ ./run_gazebo_world.sh` to launch Gazebo with `test_track.world` and the local models.

You can use the launch file to run Gazebo with the world model as well if you have sourced the new models:
`$ roslaunch gazebo_assets_drone_race test_track.world`

## Launch files
New launch file for testing the assets with Gazebo Rotors.

## Custom models
-- Aruco markers

## Aruco Markers 
Markers are made using Aruco markers with a 4x4 dictionary. Dimensions are 0.5mx0.5m

### Regular marker
Regular are gates are ID: 10.

### Landing spot / Race start 
Landing spots are ID: 7. They also function as a gate (e.g. pass 1 meter above the point) for more than 1 lap race. The landing spot pose is the same as the race start pose.

![Race Track](https://github.com/Veilkrand/gazebo_assets_drone_race/blob/master/images/default_gzclient_camera(1)-2018-12-05T13_20_42.093154.jpg)

----
##NOTES 

## Publish Waypoints to RotorS
`roslaunch gazebo_assets_race_drone test_rotors_waypoint.launch`

## Test covariances
`roslaunch rotors_simulator_demo mav_hovering_example_msf.launch`

`rostopic echo /hummingbird/odometry_sensor1/pose_with_covariance`
`rostopic echo /firefly/odometry_sensor1/pose_with_covariance`

## Send commands
`rosrun rotors_gazebo waypoint_publisher 5 -5 3 0 0 __ns:=firefly`
`rosrun rotors_gazebo waypoint_publisher 0 -0 1 0 0 __ns:=hummingbird`



