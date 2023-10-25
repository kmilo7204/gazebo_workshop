FROM nvidia/cudagl:11.0.3-base-ubuntu16.04

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

# Setup ROS kinetic sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add ROS repository key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full

# Setup ROS environment variables
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

RUN sudo apt install wget

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN curl -k 'https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/3bf863cc.pub' | apt-key add -

# Upgrade Gazebo 7.
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update && apt-get install -y \
    gazebo7 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-kinetic-navigation \
    ros-kinetic-map-server \
    ros-kinetic-move-base \
    ros-kinetic-amcl \
    ros-kinetic-openslam-gmapping \
    ros-kinetic-map-server \
    ros-kinetic-move-base \
    ros-kinetic-navigation \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-joint-state-publisher \
    ros-kinetic-joint-state-controller \
    ros-kinetic-effort-controllers \
    ros-kinetic-velocity-controllers \
    ros-kinetic-openslam-gmapping \
    ros-kinetic-joy \
    ros-kinetic-joy* \
    ros-kinetic-yocs-* \
    ros-kinetic-kobuki* \
    ros-kinetic-depthimage-to-laserscan \
    ros-kinetic-turtlebot-navigation \
    ros-kinetic-openni2-launch \
    ros-kinetic-urg-node \
    && apt-get install -y --fix-broken

# Ignition dependencies to compile plugins
RUN apt-get update && apt-get install -y \
    libignition-math4-dev

# Create a catkin workspace
RUN mkdir -p /catkin_ws/src

# Set the working directory to the catkin workspace
WORKDIR /home

# Set the entrypoint
ENTRYPOINT ["/bin/bash"]
