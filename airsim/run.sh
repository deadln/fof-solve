#!/bin/bash

dir=$( cd `dirname $0` && pwd )

if [ -z "$AIRSIM_PKG" ]; then
 AIRSIM_PKG="*"
fi

cmd=`find -L "$dir" -path "*/$AIRSIM_PKG/LinuxNoEditor/*.sh" | sort | head -n 1`
if [ -z "$cmd" ]; then
 echo "Please set correct AIRSIM_PKG environment variable "
 sleep 1
 exit
fi

c="/bin/sh $cmd -windowed $@"
$c
echo "Done: $c"
