#!/bin/bash

# Get the current directory
current_dir=$(pwd)

# Check if the current directory is parent/docker/
if [[ "$current_dir" == */docker ]]; then
    # If true, change directory and run the script
    cd ../src/segment-anything-2/checkpoints && ./download_ckpts.sh
else
    echo "The script must be run from /docker directory."
    exit 1
fi
