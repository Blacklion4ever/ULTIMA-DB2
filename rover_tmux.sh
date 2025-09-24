#!/bin/bash
SESSION="rover_tmux"

I2C_DEV="/dev/i2c-1"
ROVER_TTY="/dev/ttyTHS1"  # XBee côté rover
WS_ROOT=~/workspace/ros2_eloquent_ws

export ROS_DOMAIN_ID=0      # domaine ROS 2 identique pour tous les nodes
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1


# Attendre que le bus I2C soit prêt
echo "Waiting for I2C device $I2C_DEV..."
while [ ! -e $I2C_DEV ]; do
    sleep 1
done
echo "I2C device ready!"

while [ ! -e "$ROVER_TTY" ]; do
    echo "Waiting for $ROVER_TTY..."
    sleep 1
done
echo "tty device ready!"

# Source ROS2 et workspace
source ~/ros2_setup.sh

cd $WS_ROOT
tmux new-session -d -s $SESSION
tmux split-window -v -t $SESSION:0
tmux split-window -h -t $SESSION:0.0
tmux split-window -h -t $SESSION:0.2
tmux split-window -h -t $SESSION:0.3


# tmux send-keys -t $SESSION:0.3 "ros2 run joy joy_node" C-m
# tmux send-keys -t $SESSION:0.2 "ros2 run actuation_pkg actuation_joystick" C-m
tmux send-keys -t $SESSION:0.1 "ros2 run actuation_pkg actuators" C-m
tmux send-keys -t $SESSION:0.0 "ros2 run rover_link_pkg serial_listener" C-m

tmux send-keys -t $SESSION:0.4 "while ! ros2 topic list | grep -q "^/command/pan_tilt$"; do sleep 1; done && ros2 topic echo /command/pan_tilt" C-m
tmux select-pane -t 0.4
tmux select-layout tiled
tmux attach -t $SESSION




