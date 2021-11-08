#!/bin/bash

dir=`pwd`/`dirname $0`

source $dir/params.sh

source /opt/ros/${ros_release}/setup.bash

catkin_ws=$1
rosinstall_file=$2

rm -rf $catkin_ws
mkdir -p $catkin_ws
cd $catkin_ws

wstool init src $rosinstall_file
wstool update -t src -j4

rosdep install --from-paths src --ignore-src -y
catkin build
