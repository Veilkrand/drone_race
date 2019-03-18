This package is responsible for finding the drone's location and publishing
that location to /odometry/filtered.

# Usage

```sh
roslaunch odometry odometry.launch
rostopic echo /odometry/filtered
roslaunch odometry diagnostic.launch
```

# Strategy

This package uses three sources of state information: IMU, range finding, and gates.
The [robot_localization package](http://wiki.ros.org/robot_localization) performs
sensor fusion on those three sources to produce the /odometry/filtered topic.

