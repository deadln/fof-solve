#!/bin/bash

num=3

if [ "$1" ]; then
 num=$1
 shift
fi

AIRSIM_PKG=formation ./start.sh -n $num $@

echo "waiting  ..."
sleep 3

./examples/group.py $num

./stop.sh
