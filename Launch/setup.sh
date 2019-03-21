#!/bin/bash
#
echo "Creating catkin directory \n"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

echo "Downloading & Installing Dependencies.. \n"
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
#deactivate some dependencies
cd ~/catkin_ws
touch src/rotors_simulator/rotors_hil_interface/CATKIN_IGNORE
touch src/asctec_mav_framework/asctec_hl_interface/CATKIN_IGNORE
echo "Cloning the main repo.. \n"
cd ~/catkin_ws/src
git clone git@github.com:Veilkrand/drone_race.git