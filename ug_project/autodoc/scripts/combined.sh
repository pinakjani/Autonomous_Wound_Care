#!/bin/bash 

roslaunch autodoc detect.launch
wait 

cd ~/catkin_ws/src/ug_project/autodoc/scripts
python3 tfdetect.py
wait

rosrun autodoc wound_burn.py
wait

roslaunch autodoc detect.launch
wait 

cd ~/catkin_ws/src/ug_project/autodoc/scripts
python3 tfdetect.py
wait

rosrun autodoc clean_wound_cut.py

