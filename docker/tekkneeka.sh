#!/bin/bash

# Set a default image and container name
BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD)
MODIFIED_BRANCH_NAME=$(echo "$BRANCH_NAME" | sed 's/[^a-zA-Z0-9_\-]/-/g')
IMAGE_NAME="paradockerimage:${MODIFIED_BRANCH_NAME}"
CONTAINER_NAME="tekkneeka_container"

function build_image {
    echo "Building Docker image: $IMAGE_NAME"
    sudo docker build -t $IMAGE_NAME -f docker/Dockerfile .
}

function run_container {
    XSOCK=/tmp/.X11-unix
    XAUTH=/tmp/.docker.xauth

    # Ensure the DISPLAY variable is set
    if [ -z "$DISPLAY" ]; then
        echo "The DISPLAY environment variable is not set."
        exit 1
    fi

    # Allow local connections to the X server
    xhost +local:root

    # Create Xauthority file
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

    MODE=${1:-default}  # Mode comes first (default, sim, camera, etc.)
    CMD=${2:-/bin/bash}  # Optional command comes second

    # Define device options based on mode
    DEVICES=""
    if [ "$MODE" == "default" ]; then
        for device in /dev/video*; do
            if [ -e "$device" ]; then
                DEVICES+="--device=$device:$device "
            fi
        done
        if [ -e "/dev/ttyACM0" ]; then
            DEVICES+="--device=/dev/ttyACM0:/dev/ttyACM0 "
        fi
    elif [ "$MODE" == "camera" ]; then
        for device in /dev/video*; do
            if [ -e "$device" ]; then
                DEVICES+="--device=$device:$device "
            fi
        done
    fi


    # Run the container with different configurations based on the mode
    sudo docker run -it --rm \
        --gpus all \
        --network host \
        --ipc host \
        --volume="$(pwd):/ros_ws:rw" \
        --volume="$XSOCK:$XSOCK:rw" \
        --volume="/dev/shm:/dev/shm" \
        --volume="/dev:/dev" \
        --privileged \
        --env="DISPLAY=$DISPLAY" \
        --env="XAUTHORITY=${XAUTH}" \
        --env="QT_X11_NO_MITSHM=1" \
        --name $CONTAINER_NAME \
        $DEVICES \
        $IMAGE_NAME $CMD

    # Clean up after container exits
    xhost -local:root
}

# Command-line options
case "$1" in
    build)
        build_image
        ;;
    run)
        shift  # Remove the 'run' argument
        MODE="$1"
        shift  # Remove the mode argument
        run_container "$MODE" "$@"
        ;;
    *)
        echo "Usage: $0 {build|run [mode] [command]}"
        echo "Modes: default (all devices), sim (simulation only), camera (camera devices only)"
        exit 1
        ;;
esac