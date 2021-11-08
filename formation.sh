#!/bin/bash

num=6
pkg="formation"

source params1.sh

AIRSIM_PKG=$pkg ./start.sh -n $num --pose_list places.txt --ref_point 0,-72,0.1 $@

if [ $mode == "prof" ]; then
  suf=""
  args="--names T E C T"

  sleep 3
  ./bin/formations_gen.py iris $num tasks/formation/borders.txt tasks/formation/test_fs${suf}/ $args &
fi

read -p "Press enter to stop ..."
./stop.sh
