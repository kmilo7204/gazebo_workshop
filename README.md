# gazebo_workshop
Gazebo tutorials

## Build
Build the container with the following command:

  ```
  docker build -t ros-gazebo:16.04 .
  ```

## Launch
Launch the container with the following command:
  ```
  docker run -it \
    --privileged \
    --gpus "all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=/tmp/.docker.xauth" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/tmp/.docker.xauth:/tmp/.docker.xauth:rw" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --net=host \
    -v /home/ekumen/Camilo_Repos/Udacity-Robotics-ND/catkin_ws/src:/catkin_ws/src\
    ros-gazebo:16.04
  ```


./run.sh ~/Camilo_Repos/gazebo_workshop/Project_1-Building/ cmake ros-gazebo:16.04