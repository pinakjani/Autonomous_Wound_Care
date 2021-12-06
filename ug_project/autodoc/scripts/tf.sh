#!/bin/bash 



roslaunch autodoc detect.launch
wait 

cd ~/catkin_ws/src/ug_project/autodoc/scripts
python3 tfdetect.py
wait


