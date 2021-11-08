rosrun mavros mavwp load mission.txt
rosrun mavros mavwp show
rosrun mavros mavsys mode -c AUTO.MISSION

rosrun mavros mavsafety arm
