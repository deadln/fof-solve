if [ -z $1 ]
then
  echo "run script with number of mavros instances"
  exit 0
fi

for (( i=1 ; i<=$1; i++ ))
do
 echo "mission $i"
 rosrun mavros mavwp -n mavros$i load mission$i.txt
 rosrun mavros mavwp -n mavros$i show

 rosrun mavros mavsys -n mavros$i mode -c AUTO.MISSION
done

sleep 1

for (( i=1 ; i<=$1; i++ ))
do
 echo "arm $i"
 (rosrun mavros mavsafety -n mavros$i arm)
done
