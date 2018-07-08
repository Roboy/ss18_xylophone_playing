# ss18_xylophone_playing
This project focuses on making Roboy playing the xylophone in simulation and reality.

## Pre-requisites & Dependencies

### git 
```
#!bash
sudo apt install git
```

### ncurses
```
#!bash
sudo apt install libncurses5-dev
```

### ros
```
#!bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update

sudo apt install ros-kinetic-desktop-full
sudo apt install ros-kinetic-controller-interface ros-kinetic-controller-manager ros-kinetic-control-toolbox ros-kinetic-transmission-interface ros-kinetic-joint-limits-interface ros-kinetic-ecl-geometry ros-kinetic-gazebo-ros-control  ros-kinetic-grid-map-ros
```
We will need more packages, which we will go into later on.

### gazebo7 and gazebo-ros-pkgs
```
#!bash
sudo apt-get install gazebo7 libgazebo7-dev
sudo apt-get install ros-kinetic-gazebo-ros-pkgs
```

[bitbucket](https://bitbucket.org/osrf/gazebo_models/downloads) -> download repositor -> unzip -> move to gazebo models path
```
#!bash
cd /path/to/osrf-gazebo_models-*.zip
unzip osrf-gazebo_models-*.zip -d gazebo_models
mv gazebo_models/osrf-gazebo_models-*/* ~/.gazebo/models
```

### clone repository
```
#!bash
git clone --recursive https://github.com/Roboy/ss18_xylophone_playing.git
cd ss18_xylophone_playing
```
#### The roboy_simulation repository is private for now due to copyright reasons. Therefore you need to ask for permission to clone!

### some more stuff you might need

```bash
#!bash
sudo apt-get install libpcap0.8-dev     # package required by powerlink
sudo apt-get install protobuf-compiler  # NOTFOUND error fix: https://github.com/ethz-asl/rotors_simulator/issues/354
sudo apt-get install lib32ncurses5-dev  # fix for http://stackoverflow.com/questions/14416487/gcc-usr-bin-ld-error-cannot-find-lncurses
```

## How to build

#### Update myoFPGA path

The last thing you have to do is to update a library path. This is an absolute path and therefore, it has to be changed to your local absolute path. 
Please ensure that you are in the roboy-ros-control package and execute then the following command: 

```bash
#!bash
sed -i "s|/home/roboy/workspace/myoFPGA/myoFPGA|$(pwd)|g" src/roboy_managing_node/include/roboy_managing_node/myoMaster.hpp
```

### IMPORTANT: environmental variables and sourceing
For both build and especially running the code successfully you will need to define some env variables and source some stuff. Add the following lines to your ~/.bashrc (adjusting the paths to your system):
```
#!bash
source /usr/share/gazebo-7/setup.sh
export GAZEBO_MODEL_PATH=/path/to/ss18_xylophone_playing/src/roboy_models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/path/to/ss18_xylophone_playing/devel/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_RESOURCE_PATH=/path/to/ss18_xylophone_playing/src/roboy_models:$GAZEBO_RESOURCE_PATH
source /opt/ros/kinetic/setup.bash
source /path/to/ss18_xylophone_playing/devel/setup.bash
```
Then you can build with:
```
#!bash
source ~/.bashrc
cd path/to/ss18_xylophone_playing
catkin_make --pkg common_utilities
source devel/setup.bash
catkin_make
```
#### The first build will probably fail, as the setup bash file does not exist yet, just run:
```
#!bash
source devel/setup.bash
catkin_make
```
#### If the build fails because it is unable to locate some packages:
Most of them will be ros packages, just search them:

```
#!bash
sudo apt search <missing_package_name>
```
Which will give you a list of possible matches. If it is a ros package, make sure to install the version for ros-kinetic:
```
#!bash
sudo apt install <package_name_from_results>
```

### symlink to meshes
For gazebo to find the meshes, create a symlink:
```
#!bash
mkdir -p ~/.gazebo/models
ln -s path/to/ss18_xylophone_playing/src/roboy_models/roboy_2_0_simplified ~/.gazebo/models/
ln -s path/to/ss18_xylophone_playing/src/roboy_models/roboy_2_0_left_arm_simplified ~/.gazebo/models/
ln -s path/to/ss18_xylophone_playing/src/roboy_models/roboy_2_0_left_arm_elbow ~/.gazebo/models/
```
Add whatever other models you need in the same manner.

## How to test
```
#!bash
cd path/to/ss18_xylophone_playing/src/roboy_simulation/launch
sudo apt install ros-kinetic-rqt-rviz
```
then add the rviz plugin to rqt
make sure you add the plugins on the left in rviz and change the fixed frame to world
```
#!bash
roslaunch roboy_simulation roboy_2_0_left_arm_elbow.launch
```
You can set the target position on the server as so:
rosparam set /target_pos [1.5,0,0,0]

## How to use
You can now write your own plugins and edit the launch file to incorporate them.
In future versions you will be able to start a script that uses a midi input which will trigger movements of Roboy. 

# Further Info
For more info on the project visit our Confluence Page:
https://devanthro.atlassian.net/wiki/spaces/SS18/pages/280100944/Movement+Control
