#!/bin/bash

SESSION="station_tmux"
WS_ROOT=/home/brouk/workspace/ros2_humble_ws
STATION_TTY="/dev/ttyUSB0"  # XBee côté station

tmux new-session -d -s $SESSION
tmux split-window -v -t $SESSION:0
tmux split-window -h -t $SESSION:0.0 
tmux split-window -h -t $SESSION:0.2


tmux send-keys -t $SESSION:0.0 "cd $WS_ROOT && source /opt/ros/humble/setup.bash && ros2 run joy joy_node" C-m
tmux send-keys -t $SESSION:0.1 "cd $WS_ROOT && source install/setup.bash && ros2 run actuation_pkg actuation_joystick" C-m
tmux send-keys -t $SESSION:0.2 "cd $WS_ROOT && source install/setup.bash && ros2 run station_link_pkg command_node" C-m
tmux send-keys -t $SESSION:0.3 "cd $WS_ROOT && source install/setup.bash && ros2 run station_link_pkg supervision_node" C-m

tmux select-layout tiled
tmux attach -t $SESSION
