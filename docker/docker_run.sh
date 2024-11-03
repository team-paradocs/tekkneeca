BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD)
sudo docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v $(pwd)/src:/ros_ws/src --network host $(for device in /dev/video*; do echo --device=$device:$device; done) --device=/dev/ttyACM0:/dev/ttyACM0 --gpus all paradockerimage:${BRANCH_NAME}
