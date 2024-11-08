#!/bin/bash

xhost +

# Specify the container name or ID
CONTAINER_NAME="tekkneeka_container"

# Check if the container is running
if [ $(docker ps -q -f name=^/${CONTAINER_NAME}$) ]; then
    echo "Entering the bash shell of the container: $CONTAINER_NAME"
    docker exec -it $CONTAINER_NAME /bin/bash
else
    echo "Container '$CONTAINER_NAME' is not running."
fi

xhost -local:root