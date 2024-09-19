#!/bin/bash

# Generate a unique session name
SESSION_NAME="shiv_$(date +%s)"

# Start a new tmux session in a new terminal window
gnome-terminal -- tmux new-session -d -s "$SESSION_NAME"

# Split the screen into four quadrants
# tmux split-window -v -t "$SESSION_NAME"
tmux split-window -h -t "$SESSION_NAME":0.0
# tmux split-window -h -t "$SESSION_NAME":0.2
tmux send-keys -t "$SESSION_NAME":0.1 "ros2 run regpipe_ros pcd_regpipe" Enter
tmux send-keys -t "$SESSION_NAME":0.0 "ros2 launch paradocs_control tekkneeca.launch.py" Enter      # Top left pane

# tmux send-keys -t "$SESSION_NAME":0.2 "ifconfig" Enter
# tmux send-keys -t "$SESSION_NAME":0.3 "ping google.com" Enter 


# Attach to the tmux session
gnome-terminal -- tmux attach-session -t "$SESSION_NAME"
