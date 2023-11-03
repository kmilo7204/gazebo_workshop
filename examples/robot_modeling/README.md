# Gazebo Robot Modeling and Simulation Examples
This document provides three examples of robot modeling and simulation using URDF, Xacro, and SDF. Each example illustrates a specific step in the process.

## 1. URDF (Unified Robot Description Format)

URDF is used to describe the structure, kinematics, and visual representation of robot models in a standardized way. The example shows a simple differential drive mobile robot

- **File:** `mobile_robot.urdf`

  ```xml
  <?xml version="1.0" ?>
  <robot name="mobile_robot">
    <link name="base_link">
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0.1"/>
        <geometry>
          <cylinder length="0.2" radius="0.1"/>
        </geometry>
      </visual>
    </link>

    <link name="left_wheel">
      <visual>
        <origin rpy="0 0 0" xyz="0 0.1 0"/>
        <geometry>
          <cylinder length="0.05" radius="0.05"/>
        </geometry>
      </visual>
    </link>

    <link name="right_wheel">
      <visual>
        <origin rpy="0 0 0" xyz="0 -0.1 0"/>
        <geometry>
          <cylinder length="0.05" radius="0.05"/>
        </geometry>
      </visual>
    </link>

    <joint name="left_wheel_joint" type="continuous">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <parent link="base_link"/>
      <child link="left_wheel"/>
      <axis xyz="0 0 1"/>
    </joint>

    <joint name="right_wheel_joint" type="continuous">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <parent link="base_link"/>
      <child link="right_wheel"/>
      <axis xyz="0 0 1"/>
    </joint>
  </robot>
  ```

## 2. Xacro
Xacro is used to modularize and improve the readability of URDF files. The example demonstrates the use of Xacro to create macros for the robot's wheels and joints, making the URDF file more concise and organized:

- **File:** `mobile_robot.urdf`
  ```xml
  <?xml version="1.0"?>
  <robot name="mobile_robot">
    <xacro:include filename="wheel.xacro" />

    <link name="base_link">
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0.1"/>
        <geometry>
          <cylinder length="0.2" radius="0.1"/>
        </geometry>
      </visual>
    </link>

    <wheel name="left_wheel" xyz="0 0.1 0" />
    <wheel name="right_wheel" xyz="0 -0.1 0" />
  </robot>
  ```

- **File:** `wheel.xacro`
  ```xml
  <macro name="wheel">
    <link name="${name}">
      <visual>
        <origin rpy="0 0 0" xyz="${xyz}"/>
        <geometry>
          <cylinder length="0.05" radius="0.05"/>
        </geometry>
      </visual>
    </link>

    <joint name="${name}_joint" type="continuous">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <parent link="base_link"/>
      <child link="${name}"/>
      <axis xyz="0 0 1"/>
    </joint>
  </macro>
  ```

## 3. SDF (Simulation Description Format)
SDF files are used to describe simulation worlds, including models, lighting, and physics properties. The example creates a Gazebo world with a ground plane and the mobile robot model:
  - **File:** `simulation.world`
  ```xml
  <?xml version="1.0" ?>
  <sdf version="1.6">
    <world name="my_world">
      <include>
        <uri>model://ground_plane</uri>
      </include>

      <include>
        <uri>model://mobile_robot_model</uri>
        <pose>0 0 0.5 0 0 0</pose>
      </include>
    </world>
  </sdf>
  ```
