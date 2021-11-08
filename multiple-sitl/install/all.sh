#!/bin/bash

dir=`pwd`/`dirname $0`
cd $dir

sudo ./sudo_deps.sh
sudo ./ros/sudo_deps.sh

./prepare.sh $@
./ros/prepare.sh
