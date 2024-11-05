# Configuration file for ROS2 and custom aliases
# Gets exported to .bashrc within the container

# Source ROS2 Humble setup file
source /opt/ros/humble/setup.bash

# Set ROS2 console output format to include severity and node name
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{name}]: {message}'

# Enable colored console output for ROS2 messages
export RCUTILS_COLORIZED_OUTPUT=1

# Alias to kill all ROS2 and Gazebo processes
alias tek_kill='(ps aux | grep ros | grep -v grep && ps aux | grep gzserver | grep -v grep && ps aux | grep gzclient | grep -v grep) | awk '\''{print \$2}'\'' | xargs kill -9'

# Alias to build the workspace and source the new setup file
alias tek_install='colcon build && source install/setup.bash'

# Add Conditional Source of ROS2 workspace setup file
if [ -f "/ros_ws/install/setup.bash" ]; then
    source /ros_ws/install/setup.bash
fi
