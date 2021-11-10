#!/bin/bash

num=1
pkg="afs"

source params1.sh

AIRSIM_PKG=$pkg ./start.sh -n $num --ref_point 10,-785,0.5 --home-gps 1,1,220 $@

read -p "Press enter to stop ..."
./stop.sh
