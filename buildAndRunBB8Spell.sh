#!/bin/bash

#run catkin_make to build action message
cd /home/user/catkin_ws
catkin_make

#rerun devel script to make message available
source devel/setup.bash

#run launch file that launches action server node and action client node
roslaunch spelling_action_launch_and_call bb8_spell.launch