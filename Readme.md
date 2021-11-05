# Description
This repository contains all the relevant configurations which a user will need for successfully setting up ROS, C++ and Python (with debugging) on Visual Studio Code. Versions shown below:


# Installation Instructions
This is a quick guide on how to setup Visual studio code to work with ros and act as a feature rich IDE with features like, code highlighting, code compellation and step though debugging for both python and c++ nodes. I have provided a VM (windows only) with this already configured but i would recommend setting this up on your own VM's. 

For more detailed information and sources see the end of the guide.   
## prerequisite packages
A few packages need to be installed for VS code to work with proper debugging, open the terminal and install the following packages. 
```text
sudo apt install gcc gdb -y
```
## Installing Visual Studio Code (Windows)
1) Open the terminal and run
```text
sudo apt install snapd
sudo snap install code --classic
```
2)  Run **code** in the terminal to test visual studio is installed
## Installing Visual Studio Code (Apple silicon mac)
1) Download the ARM64 package : ```https://code.visualstudio.com/docs/?dv=linuxarm64_deb```
2) Navigate to the download directory and run the folloering command on the downloaded file :
```text
dpkg -i "YOUR DOWNLOADED FILE"

```
3) Run **code** in the terminal to test visual studio is installed

## Installing VS code extensions 
The following extensions need to be installed in visual studio, to install extensions press  **C + SHIFT +  X** or click the box icon in the left of visual studio, search up the following extensions and install them. 
```
1) C/C++
2) C++ Intellisense
3) catkin-tools
4) CMake
5) Github Pull Requests
6) Python
7) ROS
8) Command Variable
```
## Setting Up workspace environment 
The visual studio environment needs to be setup for your ros workspace.

1) In visual studio code click **open file** and navigate to your **ros workspace**, this should be the folder with your src, devel and build folder inside of it. This should open up your ros workspace in visual studio code. 
2) Crete a **.vscode** folder inside of your ros workspace. 
3) Download the following **json** files and place then in the .vscode directory 
```
Windows : DIR
Apple silicon : DIR
```
4) The next steps will take you though configuring each file.


## Setting Up 'c_cpp_properties.json'
the c_cpp_properties.json file is used to configure the c++ build environment for visual studio. 

1) Inside the file change all ocurences of **YOUR_USER** to your linux username
2) Inside the file change all ocurences of **ros_workspace** to the name of your ros workspace
## Setting Up 'launch.json'
Used to launch eater the python or the c++ node, make sure all extensions are installed to ensure this works properly. Required for both python and c++ debugging. 

**Note** : there is an odd error with the Mac versions of ros to due with the python launch section config, this can be safely ignored and has been patched for newer persons of python. 

## Setting Up 'settings.json'
1) Inside the file change all ocurences of **YOUR_USER** to your linux username
2) Inside the file change all ocurences of **ros_workspace** to the name of your ros workspace

## Setting Up 'task.json'
Used to run catkin make though visual studio code, default configuration build all code with debug enabled. 

# Using ROS in VS Code
## Commands
To open up the visual studio codes command palette (line) press ``` Ctrl + Shift + p ``` then type in the command. 

* Launching roscore : ```ROS:Start``` 
* Building code (catkin_build) :
  1) ```Tasks: Run task```
  2) ```catkin_make: build```

## Python
Debugging python file

1) Open the python file you would like to debug
2) Create a break point in the python code (click the red circle to the left of the line numbers)
3) Click run and debug button (play icon with a bug on it)
4) Select ```Python : Current file``` in the task bar (top of the left task bar with a green play button next to it)
5) Click the green play button to launch the python file 
6) (You can also press F5 if the debugger is set to python)

## C++
1) Open the c++ file you would like to debug
2) Ensure it has been built with debug enabled (default for this configuration)
3) Create a break point in the c++ code (click the red circle to the left of the line numbers)
4) Click run and debug button (play icon with a bug on it)
5) Select ```gdb : launch``` in the task bar (top of the left task bar with a green play button next to it)
6) Click the green play button to launch the c++ file 
7) (You can also press F5 if the debugger is set to c++)
# Additional Information
## Versions
1) ROS Noetic
2) Python 3.8.10
3) cmake version 3.16.3
4) gcc 9.3.0
5) Ubuntu 20.04

## Necessary VS Code Extensions
1) C/C++
2) C++ Intellisense
3) catkin-tools
4) CMake
5) Github Pull Requests
6) Python
7) ROS
8) Command Variable

## Useful Commands
### VS Code
```text
Ctrl + Shift + p
```

### Terminal
```text
source ~/ros_workspace/devel/setup.sh
catkin_init_workspace
catkin_make
```

## Useful Links
[1] : http://www.lib4dev.in/info/MrGnomes/VS_Code_ROS/i19793 "Guide of integrating ros into vs code"

[2]  : https://vitux.com/how-to-use-gdb-to-debug-programs-in-ubuntu/ "How to setup the debugger for c++"

[3]  : https://github.com/ms-iot/vscode-ros/blob/master/doc/spec/debug-ros-nodes.md "extra debug setup info"

[4]  : https://code.visualstudio.com/docs/editor/variables-reference "Used to setup config for the .json files"
