# [Tekkneeca](https://mrsdprojects.ri.cmu.edu/2024teame/)
Assistive Robot for Total Knee Arthroplasty

## Prerequisite

The host machine should be a Ubuntu distribution supported by [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).

## Installation

Clone the package
```
mkdir ~/<ros2_ws>
cd ~/<ros2_ws>/src
git clone git@github.com:team-paradocs/tekkneeca.git
```

Grant host machine run docker without sudo
```
sudo groupadd docker
sudo gpasswd -a $USER docker
newgrp docker
```

Set X11 forwarding 
```
xhost +local:docker
```

SAM checkpoints are not included, so download them manually.
``` 
./docker/get_ckpts.sh
```

Note - The script only downloads the small and large models. If you want additional models, modify segment-anything-2/checkpoints/download_ckpts.sh

## Usage

> **Note:** Run all scripts from the root directory of the repository.

### Build the Docker image

```bash
./docker/tekkneeka.sh build
```

### Run the Docker container

```bash
./docker/tekkneeka.sh run
```
If the entrypoint is enabled, it will build the ROS workspace and source the new setup file on container start.

#### Variants

- `./docker/tekkneeka.sh run sim` - Run sim version (no devices)
- `./docker/tekkneeka.sh run camera` - Run camera version (only video devices)
- `./docker/tekkneeka.sh run` - Run default version (all devices)

### Attach a terminal to the Docker container

```bash
./docker/shell.sh
```

## Troubleshooting

If you encounter "Authorization required, but no authorization protocol specified" failure, run X11 forwarding again.

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
ros2 launch paradocs_control tek_full_pilz_hybrid.launch.py
```

In another container terminal
```
ros2 run parasight host
ros2 run parasight tracker
```
