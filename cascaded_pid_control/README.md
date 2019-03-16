### cascaded_pid_control package

The cascaded pid control package is written w.r.t. MIT FlightGoggles simulator. To run it, you don't need to bring up gazebo simulation (MIT FlightGoggles is similar to gazebo).

Please note that currently the controller is not good enough to follow the trajectory commands generated from `spline_planner`.

The following instruction will tell you how to hang around in the scene, and if you insist on seeing things working together (hope that won't make you upset), it will tell you how to bring up the planner too.

#### File structure

There're 3 launch files in `cascaded_pid_control` package:

1. `cascaded_pid_control_gt.launch`

    This is used to bring up the control node and a cheating node publishing odometry message. By now, there's only position and linear velocity information in the odometry message. Angular velocity and acceleration info is yet to be added.

2. `flightgoogles_core.launch` 

    This is basically a copy of the `core.launch` from `flightgoogles` package. RViz node from the original launch file is removed.

3. `spline_planner.launch`

    This is a modified version from the original one in `drone_race_common`, to glue the planner and the controller together. The `gt_gates_publisher_node.py` in the original one is substituted with `cheat_gate_locations_node.py` in the `cascaded_pid_control` package.

####  Some scripts in this package

1. `cheat_gate_locations_node.py`

    This is the new gate publisher mentioned earlier. It publishes gate location ground truth. Instead of publishing all the gates in the scene, it let the user specify the gates in the course.

2. `set_point.py`

    This is a utility node for issuing single-point trajectory command, so you don't need to manually manipulate the drone to hang around the scene.

#### Detailed startup commands

1. Currently it is recommended you run the renderer and the ROS part of FlightGoggles separately. That would ease restarting of the simulator. It is also recommended turning on `ignore_collisions` mode.

    You need to run the following commands in two separate terms:

    Term1:

    ```shell
    rosrun flightgoggles FlightGoggles.x86_64
    ```

    Term2:

    ```shell
    roslaunch flightgoggles core.launch use_external_renderer:=1 ignore_collisions:=1
    ```

    If you're annoying about the rviz poping up everytime, you can run this instead:
    ```shell
    roslaunch cascaded_pid_control flightgoggles_core.launch use_external_renderer:=1 ignore_collisions:=1
    ```

2. Start up the cascaded pid control package in a new terminal:

    ```
    roslaunch cascaded_pid_control cascaded_pid_control_gt.launch
    ```

    After this, you're ready to use `set_point.py` script to hang around:

    "Take off":
    ```shell
    rosrun cascaded_pid_control set_point.py 0 0 3 0
    ```

    Fly to some position:
    ```shell
    rosrun cascaded_pid_control set_point.py 1.237332 9.001728 2.9625 90
    ```

    `set_point.py` command accepts 3 or 4 parameters. The first 3 parameters are the targeted location, specified in NWU manner in global `/world` frame. The last parameter specify the targeted yaw angle in degree, and if omitted, it will be set to 0, i.e., facing to the north.

    It is important to make the drone take off first before hanging around. If you try to set point before the drone is hovering, the drone is very likely to flip over. That's because there is no thrust when the drone is sitting still. It won't be able to change attitude. At the time of writing, there isn't effective way to detect whether the drone is taken off or sitting on the ground. So you need to do it manually.

3. If you want to start the planner part too, run:

    ```shell
    roslaunch cascaded_pid_control spline_planner.launch course:="2 9 13"
    ```

    The magic string `"2 9 13"` in the above command specifies the gates of course. There're many gates in the simulated scene and course is consist with some of them. The above command means you want a course with Gate2 the first, Gate9 the second and Gate13 the last. Gate definition can be found here:
    https://github.com/mit-fast/FlightGoggles/blob/master/flightgoggles/config/challenges/gate_locations.yaml

    Please remember to take off before you run this command, otherwise the drone will surely flip over.

    A kindly note (again): The controller is not good enough to follow the trajectory the planner feed.

#### TL;DR for seeing the drone going

Here's the command sequence if you're just impaitient:

```shell
roslaunch flightgoggles core.launch use_external_renderer:=1 ignore_collisions:=1
rosrun flightgoggles FlightGoggles.x86_64
roslaunch cascaded_pid_control cascaded_pid_control_gt.launch
roslaunch spline_planner planner_new.launch load_test_course:=1
```

