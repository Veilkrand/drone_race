# Installation instructions

Detailed instructions to setup a development environment for the Drone Race project

---

#### Contents

* [ROS and Catkin workspace](#ros-and-catkin-workspace)
* [Dependencies](#dependencies)
* [Flight Goggles](#flight-goggles)
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
sudo apt-get install ros-kinetic-sophus ros-kinetic-robot-localization
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
#
```

#### Usage

```shell
cd ~/catkin_ws
catkin build
source devel/setup.bash
roslaunch flightgoggles core.launch use_external_renderer:=1 ignore_collisions:=1
rosrun flightgoggles FlightGoggles.x86_64
roslaunch cascaded_pid_control cascaded_pid_control_gt.launch
roslaunch odometry odometry.launch
roslaunch spline_planner planner_new.launch load_test_course:=1
```

