#!/bin/bash
# Launch your file
# Source your workspace
cd ~/workspace/ros2_eloquent_ws
source install/setup.bash
printf "TELEOP LAUNCH"
ros2 launch communication_pkg teleop_robot.launch.py

