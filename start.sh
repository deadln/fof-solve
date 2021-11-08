#!/bin/bash

cws="$HOME/catkin_ws"

./stop.sh
sleep 0.5
./multiple-sitl/start.rb --firmware ./px4/firmware --firmware_initd ./px4 --ros_ws $cws --airsim ./airsim/run.sh --airsim_settings ./airsim/settings.json $@
