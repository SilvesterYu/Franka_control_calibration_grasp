# Franka-Control-Calibration
Franka crontol code + camera calibration code (ROS Noetic)

## Frankapy
Using frankapy controller, original code: https://github.com/iamlab-cmu/frankapy
I made minor changes

make sure to change frankapy/bash_scripts/start_control_pc.sh, 
look for the paths and specifications in the control pc, change it according to your own setup
control pc setup tutorial is included in a CMU RPAD shared Google Doc, will be released soon

Look for all places with "change this" in:

robot_controller/robot_controller.py 
frankapy/bash_scripts/start_control_pc.sh

and change it to your paths

## To Run
cd to this project's base folder

cd frankapy
bash ./bash_scripts/start_control_pc.sh -i iam-dopey

cd robot_controller
source [path to the project folder]/frankapy/catkin_ws/devel/setup.bash
python robot_controller.py