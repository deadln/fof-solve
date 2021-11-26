#!/bin/bash

num=3
pkg="afs"

source params1.sh

AIRSIM_PKG=$pkg ./start.sh -n $num --ref_point 106,505,-115 --home-gps 1,1,220 --airsim_settings ./airsim/settings_avoid_2.json $@

sleep 13

if [ $mode == "prof" ]; then
  ./bin/ca_radar.py iris $num zbot_ 30 50 &
fi
./bin/bots_2.py &

read -p "Press enter to stop ..."
./stop.sh
