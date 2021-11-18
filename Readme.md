# Description
This repository contains all the relevant configurations which a user will need for successfully setting up ROS, C++ and Python (with debugging) on Visual Studio Code. VS Code provides users with debugging and syntax suggestions/ support as well as github and ROS environment making it a powerful and effective tool for robotics software development.

# Prerequisite Installs and Setup
Before going throught the VS Code configuration steps, the user should install the following versions of the below (or similar, read documentation if unclear).

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
```text 
# add to bashrc to add workspace to source
sudo nano ~/.bashrc

# add the below to the bottom of your bashrc
source <PATH TO YOUR ROS WORKSPACE>/devel/setup.bash
```

Now install the following packages:
```text
sudo apt-get update

# Install Additional ROS Packages
sudo apt-get install -y ros-noetic-slam-gmapping ros-noetic-
controller-manager ros-noetic-joint-state-controller ros-noetic-
gazebo-ros*

# monitoring keyboard package
sudo apt-get install ros-noetic-teleop-twist-keyboard

# reminder that the below is the command to source to workspace
source <PATH TO YOUR ROS WORKSPACE>/devel/setup.bash
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
sudo mount -t http://127.0.0.1:9843/ davfs ~/Documents/VMShare (now you should see your shared folder)

# add command to bashrc file
sudo nano ~/.bashrc
echo <Ubuntu Password> | sudo -S mount -t davfs http://127.0.0.1:9843/ ~/Documents/VMShare -o username=<Host Machine Username>
```


# Setting Up ROS and Visual Studio Code
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
5) https://github.com/ms-iot/vscode-ros/blob/master/doc/spec/debug-ros-nodes.md
6) https://code.visualstudio.com/docs/editor/variables-reference
7) http://wiki.ros.org/noetic/Installation/Ubuntu
