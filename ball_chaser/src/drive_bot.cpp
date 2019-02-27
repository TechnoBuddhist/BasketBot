#include <iostream>
#include <string>  
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

/**
 * @brief Drive a robot to chase a ball or object of a specified RGB colour(command line argument) 
 *  using a ROS Service to publish 'geometry_msgs::Twist' messages.
 */

ros::Publisher rosCommandPublisher;

/** 
 * @brief This function should publish the requested linear x and angular velocities to the robot wheel joints
 *
 * After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
 */
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res){
  geometry_msgs::Twist motor_command; /**< Create a motor_command object of type geometry_msgs::Twist */

  motor_command.linear.x = static_cast<float>(req.linear_x);
  motor_command.angular.z = static_cast<float>(req.angular_z);

  rosCommandPublisher.publish(motor_command);

  ROS_INFO("Drive_Bot:- Robot commanded to move - linear_x: %0.2f, angular_z: %0.2f", motor_command.linear.x, motor_command.angular.z);

  return true;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "drive_bot");
  ros::NodeHandle rosNodeHandle;

  // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
  rosCommandPublisher = rosNodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Define a safe_move service with a handle_drive_request callback function
  ros::ServiceServer rosService = rosNodeHandle.advertiseService("/ball_chaser/command_robot", handle_drive_request);
  ROS_INFO("Drive_Bot:- Ready to send joint commands");

  ros::spin();

  return 0;
}