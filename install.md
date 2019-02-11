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
