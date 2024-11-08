# tekkneeca
Assistive Robot for Total Knee Arthroplasty

## Installation

Clone the package

```
mkdir ~/<ros2_ws>
cd ~/<ros2_ws>/src
git clone git@github.com:team-paradocs/tekkneeca.git
```

Grant docker permission
```
sudo groupadd docker
sudo gpasswd -a $USER docker
newgrp docker
```

Set X11 forwarding 
```
xhost +local:docker
```

Build the image

```
./docker/docker_build.sh
```

## Usage

Run the image with hardware
 ``` 
./docker/docker_run.sh
```

Run the image without Arduino
```
./docker/docker_run_camera_only.sh
```

Run the image without hardware
```  
./docker/docker_run_sim.sh
```  

Attach a new terminal to current container
```  
./docker/docker_terminal.sh
```

## Trouble shooting

If you encounter "Authorization required, but no authorization protocol specified" failure, run X11 forwarding again

```
xhost +local:docker
```
  
If you cannot connect to Arduino
```  
chmod 777 /dev/ttyACM0
```

## Launch the system

Inside docker container
```
ros2 launch paradocs_control tekkneeca.launch.py
```

In another container terminal
```
ros2 run regpipe_ros pcd_sampipe
```

## SAM Changelog
Added SAM (`segment-anything-2`) to the `src` folder

Checkpoints are not included so download them manually.
``` 
./docker/get_ckpts.sh
```

Note - Only downloads the small and large models. If you want additional models, modify segment-anything-2/checkpoints/download_ckpts.sh

