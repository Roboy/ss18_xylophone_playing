# ss18_xylophone_playing
This project focuses on making Roboy playing the xylophone in simulation and reality.

## Pre-requisites & Dependencies

### dependencies 
```
#!bash
sudo apt install libxml++2.6-dev 
```

### ros
```
#!bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update

sudo apt install ros-kinetic-desktop-full
sudo apt install ros-kinetic-controller-interface ros-kinetic-controller-manager ros-kinetic-control-toolbox ros-kinetic-transmission-interface ros-kinetic-joint-limits-interface ros-kinetic-ecl-geometry ros-kinetic-gazebo-ros-control  ros-kinetic-grid-map-ros ros-kinetic-rosbridge-suite ros-kinetic-trac-ik ros-kinetic-ecl-geometry ros-kinetic-gazebo-ros-pkgs ros-kinetic-moveit ros-kinetic-eus-qpoases
```

### gazebo7 models

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

## How to build

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
catkin_make
```

### symlink to meshes
For gazebo to find the meshes, create a symlink:
```
#!bash
mkdir -p ~/.gazebo/models
ln -s path/to/ss18_xylophone_playing/src/roboy_models/roboy_xylophone_left_arm ~/.gazebo/models/roboy_xylophone_left_arm
```
Add whatever other models you need in the same manner.

## How to test
```
#!bash
roslaunch roboy_controller roboy.launch
```
# Further Info
For more info on the project visit our Confluence Page:
https://devanthro.atlassian.net/wiki/spaces/SS18/pages/280100944/Movement+Control
