#!/bin/bash

if [ $# -lt 3 ]; then
    echo "Usage: $0 <project_folder> <build_method> <docker_image>"
    echo "Build methods: catkin (for catkin workspace), cmake (for CMake)"
    exit 1
fi

PROJECT_FOLDER=$1
BUILD_METHOD=$2
DOCKER_IMAGE=$3

# Additional Docker options for NVIDIA
DOCKER_OPTIONS="--privileged \
--env=DISPLAY \
--env=QT_X11_NO_MITSHM=1 \
--env=XAUTHORITY=/tmp/.docker.xauth \
--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
--volume=/tmp/.docker.xauth:/tmp/.docker.xauth:rw \
--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
--net=host"

if [ "$BUILD_METHOD" == "catkin" ]; then
    docker run -it --name gazebo-tutorials -v $PROJECT_FOLDER:/catkin_ws/src/ $DOCKER_OPTIONS $DOCKER_IMAGE
elif [ "$BUILD_METHOD" == "cmake" ]; then
    docker run -it --name gazebo-tutorials -v $PROJECT_FOLDER:/home/gazebo-project $DOCKER_OPTIONS $DOCKER_IMAGE
else
    echo "Invalid build method. Use 'catkin' for catkin workspace or 'cmake' for CMake."
fi
