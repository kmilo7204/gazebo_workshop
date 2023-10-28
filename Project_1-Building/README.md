# Project 1: Building a world in Gazebo
Create a Gazebo world which includes a custom robot model and a Plugin to print a Welcome message

<img width="682" alt="image" src="https://github.com/huuanhhuynguyen/RoboND-Build-My-World/assets/49252525/26dbd01a-d56a-48cc-b8b6-8fc08dc89565">

## Project structure
### Models
Models folder contains all the Gazebo models we created with the Gazebo model/building editor. For this specific case:

```
└── models
    └── building
        ├── model.config
        └── model.sdf
    └──robot
        ├── model.con
        └── model.sdf
```

### Scripts
Scripts folder contains all the code that will be executed in this simulation. For this specific case it is not required to have a `catkin_ws`, since there is no interaction with ROS; however, there is a simple plugin that will print a Welcome message in the console when the simulation is launched.

```
└── scripts
    └── welcome_msg.cpp
```

### World
World folder contains the Gazebo worlds, wich are made up with the previous created models.
```
└── world
    └── office.world
```

## Build
To build the script which prints the Welcome message follow the next steps:

1. Create the build folder:
    ```
    mkdir build && cd build
    ```
2. Build the code:
    ```
    cmake ../
    make
    ```
3. Add the build folder to the Gazebo plugin path:
    ```
    export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$PATH_TO_BUILD_FOLDER
    ```
For example, if running in the container `export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/gazebo-project/build`

## Launch the simulation
1. In the Repository project folder, rum the following command to launch the simulation:
    ```
    gazebo world/Office
    ```
