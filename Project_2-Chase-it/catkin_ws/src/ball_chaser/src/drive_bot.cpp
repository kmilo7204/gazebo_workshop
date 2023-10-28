#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;

// Callback function that executes whenever a drive_bot service is requested
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
    // Velocity commands are provided from the request
    geometry_msgs::Twist motor_command;
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    // Publish the velocity commands to drive the robot
    motor_command_publisher.publish(motor_command);

    // Provide a feedback message with the published velocities
    res.msg_feedback = "Publishing velocities " + std::to_string(motor_command.linear.x) + ", " + std::to_string(motor_command.angular.z);
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Register the Publisher into the Nodehandler
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Create the Service to handle the drive or movement request from the process image node
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    ros::spin();
    return 0;
}
