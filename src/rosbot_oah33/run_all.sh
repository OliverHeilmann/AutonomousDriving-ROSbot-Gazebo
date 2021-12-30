#!/bin/bash

gnome-terminal --tab -- bash -ic 'roslaunch rosbot_oah33 teleop_twist.launch'
gnome-terminal --tab -- bash -ic 'roslaunch rosbot_oah33 master.launch'
