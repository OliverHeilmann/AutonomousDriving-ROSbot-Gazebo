# Description
This repository contains the implementation of the ROSbot reactive planner. Code is written using Python, C++ and ROS. Later stages of this repository describe, in detail, how to rebuild the environment which was used for testing the system. For performance evaluation, the Gazebo simulator environment was used. The goal of this project was to autonomously drive from starting line, to finishing, as fast as possible.

_Click to watch!_
[![Watch the video](/imgs/still.png)](https://www.youtube.com/watch?v=iZE5kW37VIg)

# Highlights
Reactive planner design illustration. Specifically, this is a finite state machine model.
![](/imgs/planner.png "reactive planner")

Sensor data points outlined in RVis. Lidar data represented as yellow points and infra-red, as grey cones (two front and two rear).
![](/imgs/sensors.png "sensors")

Discretisation of Lidar data method illustrated below.
![](/imgs/lidar.png "Lidar Approach")

Examples of dithering presented below (where ROSbot gets stuck moving between two points).
![](/imgs/objectavoidance.png "object avoidance")

Examples of four routes which the ROSbot would take (those of which varied based on the starting position of the ROSbot).
![](/imgs/routes1.png "routes1")
![](/imgs/routes2.png "routes2")


# Start Commands
1) rosrun rosbot_bath run_rosbot.sh
2) rosrun rosbot_oah33 run_all.sh


------------------------------------------------------------------------
------------------------------------------------------------------------
------------------------------------------------------------------------

# Prerequisite Installs and Setup
This repository contains all the relevant configurations which a user will need for successfully setting up ROS, C++ and Python (with debugging) on Visual Studio Code. VS Code provides users with debugging and syntax suggestions/ support as well as github and ROS environment making it a powerful and effective tool for robotics software development. Before going throught the VS Code configuration steps, the user should install the following versions of the below (or similar, read documentation if unclear).

## Versions
1) ROS Noetic (see 'Setting Up Ubuntu and ROS')
2) Python 3.8.10
3) cmake version 3.16.3
4) gcc 9.3.0
5) Ubuntu 20.04 (see 'Setting Up Ubuntu and ROS')

## Necessary VS Code Extensions
1) C/C++
2) C++ Intellisense
3) catkin-tools
4) CMake
5) Github Pull Requests
6) Python
7) ROS (see 'Setting Up Ubuntu and ROS')
8) Command Variable

# Setting Up Ubuntu and ROS
## Install Ubuntu Virtual Machine
Follow these tutorials:
1) https://mac.getutm.app 
2) https://mac.getutm.app/gallery/ubuntu-20-04

## Install ROS
Follow this tutorial:
1) http://wiki.ros.org/noetic/Installation/Ubuntu
2) Run the following commands
```sh 
# Navigate to your ros workspace with src folder first
catkin_make

# add to bashrc to add ros workspace to source
sudo nano ~/.bashrc

# add the below to the bottom of your bashrc
source <PATH TO YOUR ROS WORKSPACE>/devel/setup.bash
```

Now install the following packages:
```sh
sudo apt-get update

# Install Additional ROS Packages
sudo apt-get install -y ros-noetic-slam-gmapping ros-noetic-controller-manager ros-noetic-joint-state-controller ros-noetic-gazebo-ros*

# monitoring keyboard package
sudo apt install ros-noetic-teleop-twist-keyboard

# catkin dependencies
sudo apt install python3-rosdep

# install catkin
sudo apt-get install python3-catkin-tools

# reminder that the below is the command to source to workspace
source <PATH TO YOUR ROS WORKSPACE>/devel/setup.bash
```

If any errors appear when running the `Start Commands` listed above, there might be an issue with the python linking (python vs python3). To fix this issue, do the following:

```sh
# get the path to python3
whereis python3

# use the presented path to link it with python
sudo ln -s /usr/bin/python3 /usr/bin/python
```

__IMPORTANT!!__ Note that your ROS workspace (where you call the source command to) is the directory which you should open in VS Code. For example, my workspace is called '_ros_workspace_' so I use the command 'source ros_workspace/devel/setup.bash' to point ROS to my 'setup.bash' file. Note that this file will not be visible until you build the directory yourself using _catkin_init_workspace_ and _catkin_make_.

## Clipboard and Directory Sharing with UTM:
1. Follow instructions till last step here: https://mac.getutm.app/gallery/ubuntu-20-04
2. Find Shared Directory in UTM navigator, choose your directory to share (make sure Ubuntu instance is shut off)
3. Boot up Ubuntu and run the following commands:
```text
# make directories
cd Documents && mkdir VMShare && cd VMShare

# install docking package, then mount
sudo apt-get install davfs2
sudo apt install spice-vdagent spice-webdavd
sudo mount -t http://127.0.0.1:9843/ davfs ~/Documents/VMShare (now you should see your shared folder)

# add command to bashrc file
sudo nano ~/.bashrc
echo <Ubuntu Password> | sudo -S mount -t davfs http://127.0.0.1:9843/ ~/Documents/VMShare -o username=<Host Machine Username>
```
## Installing Xmonad
Follow the following commands and then reboot. User should have the choice to boot into xmonad during login.
```text
# start by updating your apt package database:
sudo apt-get update

# to install Gnome:
sudo apt install ubuntu-gnome-desktop

# next install the xmonad window manager:
sudo apt install xmonad
```

## Install Gazebo
This step simply ensures you have up to date versions of the Ubuntu and ROS Noetic distributions.
This will take some time:

```text
sudo apt-get update
sudo apt-get dist-upgrade -y
rosdep update

sudo apt-get install -y ros-noetic-slam-gmapping ros-noetic-controller-manager ros-noetic-joint-state-controller ros-noetic-gazebo-ros* 
sudo apt-get install ros-noetic-teleop-twist-keyboard

cd ~/ros_workspace
catkin_make
source ~/ros_workspace/devel/setup.sh
```

# Setting Up ROS with Visual Studio Code
## Setting Up 'c_cpp_properties.json'
Change the following section of your json file to account your relevant directories:
```text
    "includePath": [
        "/home/heilmao/ros_workspace/devel/include/**",
        "/opt/ros/noetic/include/**",
        "/usr/include/**",
        "/home/heilmao/ros_workspace/**",
        "${workspaceFolder}/**"
    ],
```

## Setting Up 'launch.json'
Simply use the launch file provided. Note that the python version referenced in the script my have a warning associated with it. This is a bug with the current version of VS Code- simply ignore it and the debugging will run as normal.

## Setting Up 'settings.json'
Replace with relevant paths to your equivalent files.

## Setting Up 'task.json'
Copy and paste the contents of the task.json file into your '.vscode' directory as shown in this code repository if it does not already appear.

# Using ROS in VS Code
## Building ROS, VS Code Environment
You can now run CTRL+SHIFT+P, search for "Tasks: Run Task" and select the "ROS: catkinmake" task we configured. Since we set it to be the default build task with the group option you can also run that task with the shortcut CTRL+SHIFT+B. The build type is set to "Debug", so that the ROS nodes can be debugged later on.

## General Usage
Before running your ROS project, users should start _roscore_. Do this by pressing Ctrl+Shift+p and then selecting the ROS:Start option.

Program in Python and C++ as normal using VS Code. Running these projects should be done with the ROS launch files and/or shell scripts. See the 'Useful Commands 

When users wish to debug their code, they should navigate to the 'Run and Debug' tab on the left of the window. At the top, you will see two options in a dropdown box; select between Python and C++ depending on your requirement.

## Useful Commands

### VS Code
```text
Ctrl + Shift + p    # run task
Ctrl + Shift + b    # build project
```

### Terminal
```text
source ~/ros_workspace/devel/setup.sh
catkin_init_workspace
catkin_make
```

## Useful Links
1) https://mac.getutm.app 
2) https://mac.getutm.app/gallery/ubuntu-20-04
3) http://www.lib4dev.in/info/MrGnomes/VS_Code_ROS/i19793
4) https://vitux.com/how-to-use-gdb-to-debug-programs-in-ubuntu/
5) https://code.visualstudio.com/download
6) https://github.com/ms-iot/vscode-ros/blob/master/doc/spec/debug-ros-nodes.md
7) https://code.visualstudio.com/docs/editor/variables-reference
8) http://wiki.ros.org/noetic/Installation/Ubuntu
9) https://answers.ros.org/question/122021/xacro-problem-invalid-param-tag-cannot-load-command-parameter-robot_description/
10) https://github.com/richardw05/mybot_ws/issues/4
11) https://askubuntu.com/questions/942930/usr-bin-env-python-no-such-file-or-directory