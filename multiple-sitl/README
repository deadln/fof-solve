Project to run multiple px4 SITL with Gazebo on Ubuntu/Debian.

Install
=======

Download firmware sources with tag vX.Y.Z:
git clone --branch vX.Y.Z https://github.com/PX4/PX4-Autopilot.git

Update sitl_gazebo:
cd Firmware
git submodule update --init Tools/sitl_gazebo

Download multiple-sitl branch X.Y:
cd Tools
git clone --branch X.Y https://github.com/acsl-mipt/multiple-sitl.git
cd multiple-sitl/install

Install all in one:
./all.sh

or

If you want to install system packages needed for simulation:
sudo ./sudo_deps.sh

Build firmware and sitl_gazebo:
./prepare.sh

If ROS/Mavros needed:
 sudo ./ros/sudo_deps.sh
 ./ros/prepare.sh
 relogin to you terminal

 If you want to use catkin modules and have rosinstall file with modules URIs:
 ./catkin_prepare.sh CATKIN_WS_PATH ROSINSTALL_FILE

 CATKIN_WS_PATH - path where catkin workspace is created
 ROSINSTALL_FILE - path to rosinstall file

cd ..

Run
===

To see all options and default values:
./start.rb -h

To start 3 instances of Iris with xterm windows to see outputs (ROS/Mavros by default)
./start.rb -n 3 --debug

To close all processes:
./kill_sitl.sh

Start without output windows:
./start.rb -n 3

Start without ROS/Mavros:

./start.rb -n 3 --nomavros


Description
===========

Basic idea:
- for each model create its own home dir with all files needed to run in SITL mode:
    root filesystem
    rcS startup script
    additional files
 
- generate each rcS from file written for appropriate model
    change parameters and module startup options according to number of model (to eliminate conflicts with others)
    also change content according to start.rb command line arguments
- run each firmware instance in self home

- run ROS/Mavros for each firmware instance if needed

- generate Gazebo world with all model instances based on model's world (for example sitl_gazebo/worlds/iris.world)
- generate sitl_gazebo xml file with options(mavlink_tcp_port or mavlink_udp_port) for each model instance
- startup Gazebo (starts sitl_gazebo plugins)


Command line arguments:
./start.rb [options] [world_file]

world_file - force using this origin world file 
-f - choose filter
-i - px4 model, should be used if --gazebo_model did not match
-o - set option common to all models in sitl_gazebo xml file
--gazebo_model - use model and named world from known resources paths (sitl_gazebo/models/iris/iris.sdf and sitl_gazebo/worlds/iris.world for iris)

--debug - open xterm output windows for all processes
--restart - restart firmware and reset models in Gazebo world

--distance - between nearest model instances in Gazebo, 
--base_port - start count ports for models from this value
--port_step - port step for sequential model
--udp_sitl - udp exchange between px4 and sitl_gazebo

--build_label - for firmware make target (px4_sitl_default is for default)

 Paths:
 --workspace - change workspace dir
 --gazebo - additional resources for gazebo, used before sitl_gazebo files
 --firmware - path to firmware folder
 --sitl_gazebo - path to sitl_gazebo folder

 rcS startup script content:
 -r - change rate for mavlink module
 --hil_gps - disable gpssim module, set MAV_USEHILGPS parameter
 --logging - disable logger modules
 --optical_flow - leave OPTICAL_FLOW_RAD mavlink stream

 ROS/Mavros specific:
 --nomavros - do not start ROS/Mavros
 --plugin_lists - path to mavros pluginlists.yaml
 --catkin_ws - path to catkin workspace


Instances receive packets on UDP ports:
num  mavlink     mavlink(onboard)  simulator   Mavros
1    15010       15011             15019       17010
2    15020       15021             15029       17020
3    15030       15031             15039       17030
...  ...         ...               ...         ...
n    m           m+1               m+9         m+2000

where m = 15000+n*10

mavlink - waits from GCS
mavlink(onboard) - waits from onboard module, such as Mavros for example
simulator - waits sensors data from sitl_gazebo
Mavros - waits from GCS

Instances send packets to UDP ports:
num  mavlink     mavlink(onboard)
1    15015       15016
2    15025       15026
3    15035       15036
...  ...         ...
n    m+5         m+6

mavlink - sends to GCS
mavlink(onboard) - sends to onboard module, such as Mavros
