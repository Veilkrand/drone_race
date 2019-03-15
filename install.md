# Installation instructions

Detailed instructions to setup a development environment for the Drone Race project

---

#### Contents

* [ROS and Catkin workspace](#ros-and-catkin-workspace)
* [Dependencies](#dependencies)
* [Cloning this project into the workspace](#cloning-this-project-into-the-workspace)
* [Usage](#usage)

---

#### ROS and Catkin workspace

These instructions assume Ubuntu 16.04 and install ROS Kinetic.

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox autoconf
sudo rosdep init
rosdep update
```
For Ubuntu 18.1 and Ros Kinetic
```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox autoconf
sudo rosdep init
rosdep update
```

It is also recommended to install some useful extra ROS packages. These packages are mainly for helping visualizing various runtime information on RViz (if you are going to run main functionalities in headless server then you don't need to install them. Install them where you have display is enough): 
```sh
sudo apt-get install ros-kinetic-jsk-visualization
```

Next create a Catkin workspace.

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

#### Dependencies

```sh
cd ~/catkin_ws/src
wstool init
wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
wstool merge rotors_hil.rosinstall
wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_demos.rosinstall
wstool merge rotors_demos.rosinstall
wstool update
git clone git@github.com:ethz-asl/ethzasl_msf.git
git clone git@github.com:ethz-asl/glog_catkin.git
git clone git@github.com:catkin/catkin_simple.git
sudo pip install scipy
```

Deactivate some dependencies that are only needed on a real drone and which would otherwise
trigger compilation errors.

```sh
cd ~/catkin_ws
touch src/rotors_simulator/rotors_hil_interface/CATKIN_IGNORE
touch src/asctec_mav_framework/asctec_hl_interface/CATKIN_IGNORE
```

#### Flight Goggles

We're still working on the transition from RotorS to Flight Goggles. These instructions get the
Flight Goggles simulator running, assuming the previous dependencies have already been installed.

```sh
cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/gflags_catkin.git
wstool merge https://raw.githubusercontent.com/mit-fast/FlightGoggles/master/flightgoggles.rosinstall
wstool update
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro kinetic --skip-keys="rotors_hil_interface asctec_hl_interface asctec_hl_firmware" -y
sudo apt install -y libzmqpp-dev libeigen3-dev
sudo apt install -y libvulkan1 mesa-vulkan-drivers vulkan-utils
catkin build
source devel/setup.bash
sudo apt-get install nvidia-384 nvidia-modprobe
```

Run the simulator with either of these options:

* Manual control with joystick: `$ roslaunch flightgoggles teleopExample.launch`
* Automated control: `$ roslaunch flightgoggles core.launch`

For manual control, you need to find the tiny "keyboard" window and make sure it is the active window.
Hold down the spacebar at all times and use the following keys to control: ASDW for thrust and yaw
rate, JKLI for roll and pitch rate.

#### Cloning this project into the workspace

```sh
cd ~/catkin_ws/src
git clone git@github.com:Veilkrand/drone_race.git
```

#### Usage

First build the project.

```sh
cd ~/catkin_ws
catkin build
```

Two launch scripts must be run together in separate terminals. The first launches the simulator
and sets up the world with a stationary quadcopter. The second controls the quadcopter's movement
in the world.

First launch the simulator. The non-camera version is recommended for better performance because
we aren't actually using the camera for anything yet. This is a slow step, so wait for it. If
you see errors such as "unable to wake gazebo" or "failed to acquire lock" just kill everything
(including all gazebo windows) and try again. Both of these are expected, intermittent errors.

```sh
cd ~/catkin_ws
source devel/setup.bash
roslaunch gazebo_assets_drone_race rotors_msf_test_track.launch
```

Alternatively, if you need the camera, use this command to launch the simulator.

```sh
cd ~/catkin_ws
source devel/setup.bash
roslaunch gazebo_assets_drone_race test_rotors_msf_camera.launch
```

After you see the stationary quadcopter and no errors in the console used to launch the simulator,
the next step is to start another node in a separate console to control the quadcopter.

```sh
cd ~/catkin_ws
source devel/setup.bash
rosrun waypoint_controller control_node.py _target_exit_speed:=1
```

The following two commands are for visualization and not required.

```sh
rosrun drone_map_builder gt_gates_publisher_node.py $ rosrun drone_map_builder observed_gates_map_node.py
```

```sh
rviz -d $(rospack find drone_map_builder)/configs/default.rviz
```

### About cascaded_pid_control package

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
rosrun cascaded_pid_control set_point.py 0 0 3 0
rosrun cascaded_pid_control set_point.py 0 0 3 90
rosrun cascaded_pid_control set_point.py 1.237332 9.001728 2.9625 90
```

If you want to see planner and controller working together badly, run:

```shell
roslaunch cascaded_pid_control spline_planner.launch course:="2 9 13"
```
