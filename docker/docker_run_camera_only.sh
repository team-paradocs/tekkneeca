BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD)
MODIFIED_BRANCH_NAME=$(echo "$BRANCH_NAME" | sed 's/[^a-zA-Z0-9_\-]/-/g')
sudo docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v $(pwd)/src:/ros_ws/src --network host $(for device in /dev/video*; do echo --device=$device:$device; done) --gpus all paradockerimage:${MODIFIED_BRANCH_NAME}