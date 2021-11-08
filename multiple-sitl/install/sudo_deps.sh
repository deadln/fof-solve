#!/bin/bash

source params.sh

#repos

echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

apt update

#gazebo
apt install gazebo${gz_ver} libgazebo${gz_ver}-dev -y

#sitl_gazebo
apt install libopencv-dev libeigen3-dev protobuf-compiler libprotobuf-dev libprotoc-dev libgstreamer1.0-dev -y

#px4
apt install git wget zip cmake build-essential genromfs -y
apt install python3-jinja2 python3-toml python3-numpy python3-empy python3-genmsg python3-packaging python3-yaml -y

#start script
apt install ruby xterm -y

#clean
apt clean
