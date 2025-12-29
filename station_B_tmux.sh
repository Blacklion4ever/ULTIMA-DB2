#!/bin/bash

SESSION="station_B_tmux"
WS_ROOT=$HOME/workspace/ros2_humble_ws
STATION_TTY="/dev/ttyUSB0"  # XBee côté station

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

tmux new-session -d -s $SESSION
tmux split-window -v -t $SESSION:0
tmux split-window -h -t $SESSION:0.0
tmux split-window -h -t $SESSION:0.2
tmux split-window -v -t $SESSION:0

tmux send-keys -t $SESSION:0.0 "cd $SCRIPT_DIR && bash ./monado_fullscreen.sh" C-m
tmux send-keys -t $SESSION:0.1 "cd $WS_ROOT && source install/setup.bash && ros2 run xr_cam_pkg xr_cam_passthrough_node" C-m
tmux send-keys -t $SESSION:0.2 "cd $WS_ROOT && source install/setup.bash && ros2 run sensor_pkg video_node" C-m
tmux send-keys -t $SESSION:0.3 "cd $WS_ROOT && source /opt/ros/humble/setup.bash && ros2 run rqt_image rqt_image" C-m
tmux send-keys -t $SESSION:0.4 "bash" C-m

tmux attach -t $SESSION
