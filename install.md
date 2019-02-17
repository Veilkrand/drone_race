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

#### About cascaded_pid_control package

The cascaded pid control package is written w.r.t. MIT FlightGoggles simulator. To run it, you don't need to bring up gazebo simulation (MIT FlightGoggles is similar to gazebo).

cascaded_pid_control package contains 2 node:

1. `cascaded_pid_control` This is the main node implementing a cascaded pid controller.
2. `cheat_node` This is a node turning groundtruth pose information into `nav_msgs/Odometry`. Note that this node use ground truth information and is not allowed to use in the final solution (thus it is named with `cheat`).

There's a launch file that can start the two nodes in one command. After building and sourcing the full workspace, run:

```sh
$ roslaunch cascaded_pid_control cascaded_pid_control_gt.launch
```

If you want to use estimated odometry instead of the cheating ground truth, you can start the `cascaded_pid_control` node only and remap the input topic to the actual `nav_msgs/Odometry` message:

```sh
$ rosrun cascaded_pid_control cascaded_pid_control /CascadedPidControl/rateThrust:=/uav/input/rateThrust /CascadedPidControl/odometry:=/the/actual/odometry/topic
```

Currently estimated odometry is not ready so you can only work with the cheat odometry.
