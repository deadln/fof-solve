rosrun mavros mavsys mode -c AUTO.TAKEOFF
rosrun mavros mavsafety arm

sleep 10

rosrun mavros mavsys mode -c AUTO.LAND
