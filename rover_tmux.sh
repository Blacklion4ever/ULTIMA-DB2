#!/bin/bash

SESSION="rover_tmux"
WS_ROOT=/home/ultima/workspace/ros2_eloquent_ws
ROVER_TTY="/dev/ttyTHS1"  # XBee côté rover

tmux new-session -d -s $SESSION

# --- Fenêtre 0 : Serial Listener ---
tmux rename-window -t $SESSION:0 'Serial Listener'
tmux send-keys -t $SESSION:0 "cd $WS_ROOT && source install/setup.bash && ros2 run rover_link_pkg serial_listener --ros-args -p serial_port:=$ROVER_TTY" C-m

# --- Fenêtre 1 : Affichage Pan/Tilt ---
tmux split-window -v -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "cd $WS_ROOT && source install/setup.bash && ros2 run actuation_pkg actuators" C-m

# --- Fenêtre 2 : Affichage Steering/Drive ---
tmux split-window -h -t $SESSION:0.1
tmux send-keys -t $SESSION:0.2 "cd $WS_ROOT && source install/setup.bash && ros2 topic echo /actuator/drive" C-m

tmux select-layout tiled
tmux attach -t $SESSION
