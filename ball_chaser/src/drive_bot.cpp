#include <iostream>
#include <string>  
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

/**
 * @brief Drive a robot to chase a ball or object of a specified RGB colour(command line argument) 
 *  using a ROS Service to publish 'geometry_msgs::Twist' messages.
 * 
 * @author Ron Johnson
 * @date 10/02/2019
 */

ros::Publisher motor_command_publisher; /**< ROS::Publisher motor commands */

// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res){
  // Create a motor_command object of type geometry_msgs::Twist
  geometry_msgs::Twist motor_command;

  // Set wheel velocities
  motor_command.linear.x = static_cast<float>(req.linear_x);
  motor_command.angular.z = static_cast<float>(req.angular_z);

  // Publish angles to drive the robot
  motor_command_publisher.publish(motor_command);

  // Return a response message
  ROS_INFO("Drive_Bot:- Robot commanded to move - linear_x: %0.2f, angular_z: %0.2f", motor_command.linear.x, motor_command.angular.z);

  return true;
}


int main(int argc, char** argv){
  // Initialize a ROS node
  ros::init(argc, argv, "drive_bot");

  // Create a ROS NodeHandle object
  ros::NodeHandle nh;

  // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
  motor_command_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Define a safe_move service with a handle_drive_request callback function
  ros::ServiceServer service = nh.advertiseService("/ball_chaser/command_robot", handle_drive_request);
  ROS_INFO("Drive_Bot:- Ready to send joint commands");

  // Handle ROS communication events
  ros::spin();

  return 0;
}