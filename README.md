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

### roboticsLibrary
https://github.com/Roboy/rl

### clone repository
```
#!bash
git clone --recursive https://github.com/Roboy/ss18_xylophone_playing.git
cd ss18_xylophone_playing
```

As parts of this project are still being worked on you need specific branches for it to work. Even then the point where we stopped developing is given by the specific commits. It might sitll work on HEAD tho.
Use the devel branch on this repository.

Submodule -> Branch -> Commit
- common_utils master HEAD
- roboy_communication roboticsLibrary 6ddc92e44c79251f530580ea9ad5e74624532405
- roboy_controller roboticsLibrary 6e28d176f29e164dcbf950c42b6d4cdf59f2b4b8
- roboy_models devel 26a160ad8a6d48277e76705757fd9cce649baa1e
- roboy_moveit_configs master HEAD
- roboy_rqt_plugins master HEAD
- roboy_simulation roboticsLibrary 34ee80505d507a6cf5d54763746db4508ec6a2c9

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
As ROS has so many components it might be that we missed some packages. If the build fails it is very likely that you are just missing a dependency. try and find out what you are missing and use 
```
#!bash
sudo apt search
```
plus some parts of that dependecy name (e.g. just the name without the version) to find that stuff.

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
in a new terminal:
```
#!bash
rviz&
```
Now wait until u see waiting for input in your terminal and use e.g.
```
#!bash
rosparam set /key 'key_F'
```
Note that not all the keys are reachable with just one arm, you can check which ones have a solution in the first terminal window. During the init phase a list of keys will be printed.

Once you see roboy has reached the target position use
```
#!bash
rosparam set /roboy_state 'done'
```
to let him know :).

# Further Info
For more info on the project visit our Confluence Page:
https://devanthro.atlassian.net/wiki/spaces/SS18/pages/280100944/Movement+Control
