# Add a Gazebo Plugin
In our previous example, we created a basic mobile robot model using URDF and Xacro, defining its structure and appearance. While the robot was a static model, we can make it come to life by adding functionality with a Gazebo plugin. In this document, we'll explore the process of enhancing our robot's capabilities using a plugin.

## The Power of Gazebo Plugins
Gazebo plugins are modules that extend the functionality of the Gazebo simulator. They allow us to add various features and behaviors to our simulated robots. In this case, we'll use a **controller plugin** to enable our robot to move and interact with its environment.

### Controller Plugin
- **Role:** A controller plugin is responsible for controlling the robot's behavior. It can actuate joints, manage sensor data, and implement control algorithms to make the robot move in a simulated environment.

- **What We Want to Achieve:** We want to give our mobile robot the ability to move, which is a crucial aspect of robotics simulations. To do this, we'll use a controller plugin that mimics the behavior of a real robot's motors and wheels.

## Plugin Configuration

Before we dive into the code, we need to configure the plugin for our robot. In our URDF/Xacro files, we defined the robot's joints and links. Now, we will link the plugin to the appropriate components.

### Sample Plugin Configuration
This section of the code can be added to the `mobile_robot.urdf` file or create a new file with `.gazebo` extension and then import in the same file.

```xml
<robot name="mobile_robot">
  <!-- ... (Previous robot description) ... -->

  <gazebo>
    <plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <publishWheelTF>true</publishWheelTF>
      <publishOdomTF>true</publishOdomTF>
    </plugin>
  </gazebo>
</robot>
```