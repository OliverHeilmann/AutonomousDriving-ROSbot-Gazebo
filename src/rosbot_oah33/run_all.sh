#!/bin/bash

gnome-terminal -- 'rosrun rosbot_bath run_rosbot.sh'
gnome-terminal -- 'rosrun rosbot_oah33 run_teleop_twist.sh'
gnome-terminal -- 'rosrun rosbot_oah33 run_master.sh'