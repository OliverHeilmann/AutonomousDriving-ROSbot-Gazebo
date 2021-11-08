# Description
This repository contains all the relevant configurations which a user will need for successfully setting up ROS, C++ and Python (with debugging) on Visual Studio Code. Versions are shown below:


# Installation Instructions
This is a quick guide on how to set up Visual studio code to work with ros and act as a feature-rich IDE with features like, code highlighting, code compellation and step though debugging for both python and c++ nodes. 

I have provided a VM (windows only) with this already configured but I would recommend setting this up on your own VM's.
The link to the VM will be posted as a reply to this thread (once the VM has been uploaded). 

For more detailed information and sources see the end of the guide.   

## prerequisite packages
A few packages need to be installed for VS Code to work with proper debugging, open the terminal, and install the following packages. 
```text
sudo apt install gcc gdb -y
```
## 🖥️ Installing Visual Studio Code (Windows)
1) Open the terminal and run
```text
sudo apt install snapd
sudo snap install code --classic
```
2)  Run **code** in the terminal to test visual studio is installed
## 🍎 Installing Visual Studio Code (Apple silicon mac)
1) Download the ARM64 package: ```https://code.visualstudio.com/docs/?dv=linuxarm64_deb```
2) Navigate to the download directory and run the following command on the downloaded file :
```text
dpkg -i "YOUR DOWNLOADED FILE"

```
3) Run **code** in the terminal to test visual studio is installed

## Installing VS code extensions 
The following extensions need to be installed in visual studio, to install the extensions press  **C + SHIFT +  X** or click the box icon in the left of visual studio, search up the following extensions and install them. 
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
The visual studio environment needs to be set up for your ros workspace.

1) In visual studio code click **open file** and navigate to your **ros workspace**, this should be the folder with your src, devel, and build folder inside of it. This should open up your ros workspace in visual studio code.
    
2) Check the **.vscode** folder is created inside of your ros workspace it is a hidden folder to be sure to enable show hidden folders. If it is not created create it. 
   
3) Download the following **json** files and place them in the .vscode directory 
```
🖥️ Windows: https://computingservices-my.sharepoint.com/:u:/g/personal/je642_bath_ac_uk/EZW4Pn40SLxHiop4Xj_0qwUB7aeYOHcoL9i8uKKS9teE4Q?e=JotHpo
🍎 Apple silicon: https://computingservices-my.sharepoint.com/:u:/g/personal/je642_bath_ac_uk/ESBLm7QjBfVBiMkdhDgf4SYBphFIqsPqP1D3vzIDrCT_zg?e=FOZH0R
```
4) Make sure the correct python version is set, (click the blue python box at the bottom of vs code)

```
🖥️ Windows: 2.7.*
🍎 Apple silicon: 3.8.*
```

## Setting Up 'c_cpp_properties.json'
the c_cpp_properties.json file is used to configure the c++ build environment for visual studio. 

1) Inside the file change all occurrences of **YOUR_USER** to your Linux username
2) Inside the file change all occurrences of **ros_workspace** to the name of your ros workspace
## Setting Up 'launch.json'
Used to launch either the python or the c++ node, make sure all extensions are installed to ensure this works properly. Required for both python and c++ debugging. 

**Note** : there is an odd error with the Mac versions of ros to due with the python launch section config, this can be safely ignored and has been patched for newer persons of python. 

## Setting Up 'settings.json'
1) Inside the file change all occurrences of **YOUR_USER** to your Linux username
2) Inside the file change all occurrences of **ros_workspace** to the name of your ros workspace

## Setting Up 'task.json'
Used to run catkin make though visual studio code, default configuration build all code with debugging enabled. 

# Using ROS in VS Code
## Commands
To open up the visual studio codes command palette (line) press ``` Ctrl + Shift + p ``` then type in the command. 

* Launching roscore : ```ROS:Start``` 

* Building code (catkin_build):
  
  1) ```Tasks: Run task```
   
  2) ```catkin_make```
   
  3) ```catkin_make: build```

## Python Debug / Run

1) Open the python file you would like to debug in visual studio.
   
2) Create a breakpoint in the python code (click the red circle to the left of the line numbers)
   
3) Click the run and debug button (play icon with a bug on it)
   
4) Select ```Python: Current file``` in the taskbar (top of the left taskbar with a green play button next to it)
   
5) In the Run and Debug box at the top of visual studio code, select ```Python: current file```
   
6) Click the green play button to launch the python file 
   
7) (You can also press F5 if the debug box is set to python)

## C++ Debug / run 
1) Open the c++ file you would like to debug in visual studio.
   
2) Ensure it has been built with debugging enabled (default for this configuration)
   
3) Create a breakpoint in the c++ code (click the red circle to the left of the line numbers)
   
4) Click the run and debug button (play icon with a bug on it)
   
5) In the Run and Debug box at the top of visual studio code, select ```(gdb) Launch```
   
6) Click the green play button to launch the c++ file 
   
7) (You can also press F5 if the debugger is set to c++)

# Additional Information

Thanks to Oliver Heilmann for help creating this guide especially around the M1 macs. 


## Sources  
[1] : http://www.lib4dev.in/info/MrGnomes/VS_Code_ROS/i19793 "Guide of integrating ros into vs code"

[2]  : https://vitux.com/how-to-use-gdb-to-debug-programs-in-ubuntu/ "How to setup the debugger for c++"

[3]  : https://github.com/ms-iot/vscode-ros/blob/master/doc/spec/debug-ros-nodes.md "extra debug setup info"

[4]  : https://code.visualstudio.com/docs/editor/variables-reference "Used to setup config for the .json files"
