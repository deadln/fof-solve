#!/bin/bash

dir=`pwd`/`dirname $0`/..

label="default"
fw_dir=$dir/../../

if [ "$1" ]; then
  label=$1
fi

if [ "$2" ]; then
  fw_dir=$2
fi

#px4 + sitl_gazebo

cd $fw_dir
make clean
make px4_sitl_$label
make px4_sitl_$label sitl_gazebo
