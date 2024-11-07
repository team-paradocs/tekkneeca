#!/bin/bash

# Source existing bashrc
source /opt/ros/humble/setup.bash
source ~/.bashrc

# Check if the workspace is built
if [ ! -d "/ros_ws/build" ]; then
    echo "Building the ROS workspace"
    cd /ros_ws && colcon build
fi

# Source the workspace
source /ros_ws/install/setup.bash

# Run the entrypoint
exec "$@"