#!/bin/bash

current_dir=$(pwd)

if [[ "$current_dir" == */tekkneeca ]]; then
    cd src/segment-anything-2/checkpoints && ./download_ckpts.sh
else
    echo "The script must be run from root directory."
    exit 1
fi
