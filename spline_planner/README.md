### About the trajectory visualizer

`trajectory_visualizer.py` will subcribe to topic `/traj_viz/trajectory` and turn it into `MarkerArray`s messages so that the the path can be shown in rviz. Currently it will visualize:

1. the trajectory. it is done by connecting points in the `transforms[0]` field of `trajectory_msgs/MultiDOFJointTrajectory` as a line list.
2. the linear velocity at points in the trajectory. `velocities[0].linear` field of `trajectory_msgs/MultiDOFJointTrajectory` is used. Velocities are shown as an arrow in rviz. The orientation of the arrow is the direction of the velocity and the length of the arrow is scaled according the the magnitude of the velocity.

By default, when launching `solution.launch`, trajectory visualizer will be configured to remap `/traj_viz/trajectory` as `/firefly/command/trajectory`. If you want do run it separately or you want to subscribe to other topics, run:

```shell
$ rosrun spline_planner trajectory_visualizer.py /traj_viz/trajectory:=/actual/trajectory/topic
```