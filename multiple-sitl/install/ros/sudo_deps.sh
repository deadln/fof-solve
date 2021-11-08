#!/bin/bash

dir=`pwd`/`dirname $0`

source $dir/../params.sh

#ROS

echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

apt update
apt install ros-${ros_release}-ros-base -y

#mavros
apt install ros-${ros_release}-mavros -y
# python-prettytable -y
/opt/ros/${ros_release}/lib/mavros/install_geographiclib_datasets.sh

#catkin
apt install python3-rosinstall python3-rosinstall-generator python3-rosdep python3-catkin-tools python3-osrf-pycommon -y

#clean
apt clean

#init
rosdep init
