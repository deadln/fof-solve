#!/bin/bash

num=3
pkg="race"

source params1.sh

AIRSIM_PKG=$pkg ./start.sh -n $num --pose_list places.txt --ref_point 0,0,0.5 $@

if [ $mode == "prof" ]; then
  suf=""

  sleep 3

  args=""
  if [ "$num" == "6" ]; then
    args="--walls tasks/race/test_ws${suf}/"
  fi

  ./bin/path_gen.py iris $num tasks/race/centrals${suf}.txt 20 ${args} &
fi

read -p "Press enter to stop ..."
./stop.sh
