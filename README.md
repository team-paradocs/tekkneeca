# tekkneeca
Assistive Robot for Total Knee Arthroplasty

## Installation

- Clone the package

        cd ~/<ros2_ws>/src
        git clone git@github.com:team-paradocs/tekkneeca.git

- Build the packages

        cd ~/<ros2_ws>
        colcon build

- source environment

        source ~/<ros2_ws>/install/setup.bash

## With Docker

- Clone the package

        cd ~/<ros2_ws>/src
        git clone git@github.com:team-paradocs/tekkneeca.git

- Grant docker permission

        sudo groupadd docker
        sudo gpasswd -a $USER docker
        newgrp docker

- X11 Forwarding 

        xhost +local:docker

- Usage

  build the image
  
        ./docker_build.sh
  
  run the image with hardware
  
        ./docker_run.sh
  
  run the image without hardware
  
        ./docker_run_sim.sh
  
  attach a new terminal to current container
  
        ./docker_terminal.sh

- Trouble shooting

  If you encounter "Authorization required, but no authorization protocol specified" failure, run X11 forwarding again

  If you cannot connect to Arduino, chmod 777 /dev/ttyACM0

## SAM Changelog
Added SAM (`segment-anything-2`) to the `src` folder

Checkpoints are not included so download them manually.
 
        ./docker/get_ckpts.sh
Note - Only downloads the small and large models. If you want additional models, modify segment-anything-2/checkpoints/download_ckpts.sh

Update build and runs scripts to run from root dir. So now do - 

        ./docker/docker_build.sh
        ./docker/docker_run.sh


