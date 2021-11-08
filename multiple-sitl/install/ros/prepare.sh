#!/bin/bash

dir=`pwd`/`dirname $0`

source $dir/../params.sh

#ROS

rosdep update

str="source /opt/ros/${ros_release}/setup.bash"
rcfile=~/.bashrc

grep "$str" $rcfile >/dev/null
if [ $? -eq 1 ];then
  echo $str >> $rcfile
  source ~/.bashrc
fi
