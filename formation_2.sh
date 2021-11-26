#!/bin/bash

num=12
pkg="formation"

source params1.sh

if [ $mode == "prof" ]; then
  num=24
fi

AIRSIM_PKG=$pkg ./start.sh -n $num --pose_list places.txt --ref_point 0,-72,0.1 -f EKF2_REF_SET=1 $@

if [ $mode == "prof" ]; then
  suf="_2/$num"
  args="--names C S P A"

  sleep 3
  ./bin/formations_gen.py iris $num tasks/formation/borders.txt tasks/formation/test_fs${suf}/ $args &
fi

read -p "Press enter to stop ..."
./stop.sh
