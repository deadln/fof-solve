#!/bin/bash

GAZEBO_MASTER_IP=127.0.0.1
GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345

export GAZEBO_MASTER_IP
export GAZEBO_IP=$GAZEBO_MASTER_IP

source /usr/share/gazebo/setup.sh

echo $*
$*
