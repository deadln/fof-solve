###   SET YOUR CONFIGURATION   ###
#gazebo server ip
GAZEBO_MASTER_IP=...
#our host ip
GAZEBO_IP=...
#path to sitl_gazebo models
MODELS=...


export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$MODELS

export GAZEBO_MASTER_IP
export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345
export GAZEBO_IP

gzclient --verbose
