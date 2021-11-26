#!/bin/bash

num=12
pkg="race_2"

source params1.sh

AIRSIM_PKG=$pkg ./start.sh -n $num --pose_list places.txt --ref_point 0,0,0.5 -f EKF2_REF_SET=1 $@

if [ $mode == "prof" ]; then
  sleep 3

  ./bin/path_gen.py iris $num tasks/race/centrals_2.txt 20 &
fi

read -p "Press enter to stop ..."
./stop.sh
