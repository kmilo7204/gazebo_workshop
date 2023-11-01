FROM ubuntu:20.04

# Set non-interactive mode during installation
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install essential packages
RUN apt-get update && apt-get install -y \
    sudo \
    gnupg \
    lsb-release \
    curl

# Set up the ROS keys
RUN curl -sSL http://packages.ros.org/ros.key | apt-key add -
# Add the ROS Noetic repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Install ROS Noetic
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full

# Setup ROS environment variables
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Fix broken dependencies
RUN apt-get install -y --fix-broken

# Clean up
RUN apt-get clean
RUN rm -rf /var/lib/apt/lists/*

# Create a catkin workspace
RUN mkdir -p /catkin_ws/src

# Set the working directory to the catkin workspace
WORKDIR /home

# Set the entrypoint
ENTRYPOINT ["/bin/bash"]

# # Expose Gazebo default port
# EXPOSE 11345

# # Start a shell by default
# CMD ["bash"]
