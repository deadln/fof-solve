#!/bin/bash

cd `dirname $0`
dir=`pwd`

git pull

if [ "$1" == "update" ]; then
 exit
fi

if [ "$1" != "prepare" ]; then
  sudo ./multiple-sitl/install/ros/sudo_deps.sh
  cat ./deps/apt.txt | sudo xargs apt install -y

  while read -a p; do
   pip install $p
  done < ./deps/pip.txt
fi

cws="$HOME/catkin_ws"
rosinstall_file=""

if [ ! -d $cws ];then
  ./multiple-sitl/install/catkin_prepare.sh $cws $rosinstall_file
fi

srcs="/opt/ros/noetic/setup.bash $cws/devel/setup.bash"
rcfile=~/.bashrc

for s in $srcs
do
  str="source $s"
  grep "$str" $rcfile >/dev/null
  if [ $? -eq 1 ];then
    echo $str >> $rcfile
    source ~/.bashrc
  fi
done
