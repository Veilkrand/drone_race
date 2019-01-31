# Gazebo world file objects to json file

Read objects from a ROS Gazebo world file parsing a name rule and extract pose and marker value ID from the name string using regex. Use run `test.bash` for an example of use.

## Example
`$ python world_objects_to_file.py ../../worlds/test_track.world aruco_4x4 'aruco_4x4_(\d\d?)'`

It will extract all model object of the world file `../../worlds/test_track.world` which name starts with `aruco_4x4` and use the marker value id extracted from the model name using a regular expresion `aruco_4x4_(\d\d?)` In this case it will get or or two digits after `aruco_4x4_`.
