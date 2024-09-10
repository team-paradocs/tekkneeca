# Use NVIDIA's official CUDA base image (Ubuntu 22.04, CUDA 11.x)
FROM nvidia/cuda:11.7.1-cudnn8-runtime-ubuntu22.04

# Set environment variables for non-interactive installations
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /ros_ws

# Update the system and install essential dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    git \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Setup ROS 2 Humble repository
RUN  apt update &&  apt install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc |  apt-key add - && \
     sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Humble (Desktop Full Installation)
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-rosdep2

RUN apt-get install -y \
    python3-pip \
    python3-nose \
    python3-colcon-common-extensions \
    python3-vcstool \
    python-is-python3

RUN apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-control-toolbox \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2

# Install MoveIt for ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-setup-assistant ros-humble-gazebo-ros2-control

RUN apt install -y ament-cmake-nose ros-humble-gazebo-ros-pkgs vim-gtk3


# Initialize rosdep
# RUN rosdep init
RUN rosdep update

RUN pip install serial

# Source ROS 2 setup file
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# RUN rosdep install --from-paths src --ignore-src -r -y

# Install x11 dependencies for GUI applications
RUN apt-get install -y libgl1-mesa-glx libxrender1 libxext6 libxtst6


# Set the default command to run when starting the container
CMD ["/bin/bash"]
