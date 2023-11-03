FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive

# Install essential packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    gnupg2 \
    lsb-release \
    sudo \
    wget \
    xterm

# Set up the ROS keys
RUN curl -sSL http://packages.ros.org/ros.key | apt-key add -
# Add the ROS Noetic repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Install ROS Noetic
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full

# Setup ROS environment variables
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install Gazebo
RUN apt-get update && apt-get install -y \
    gazebo11

# Ignition dependencies to compile plugins
RUN apt-get update && apt-get install -y \
    libignition-math4-dev

# Create a catkin workspace
RUN mkdir -p /catkin_ws/src

# Set the working directory to the catkin workspace
WORKDIR /home

# Set the entrypoint
ENTRYPOINT ["/bin/bash"]
