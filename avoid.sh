#!/bin/bash

num=1
pkg="afs"

source params1.sh

AIRSIM_PKG=$pkg ./start.sh -n $num --ref_point 110,520,-100 --home-gps 1,1,220 --airsim_settings ./airsim/settings_avoid.json $@

sleep 3

if [ $mode == "prof" ]; then
  ./bin/ca_radar.py iris $num zbot_ 16 50 &
fi
./bin/bots.py &

read -p "Press enter to stop ..."
./stop.sh
