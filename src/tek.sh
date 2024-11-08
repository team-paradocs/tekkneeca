#!/bin/bash

# Function to handle SIGINT (Ctrl+C)
cleanup() {
    echo "Terminating script..."
    pkill -P $$  # Kill all child processes
    exit 0
}

# Trap SIGINT and call the cleanup function
trap cleanup SIGINT

# Run the ROS 2 launch file and log output
ros2 launch paradocs_planning test_continuous_motion_clean.launch.py | tee run_log.txt
