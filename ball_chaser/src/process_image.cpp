#include <iostream>
#include <string>
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

/**
 * @brief Search for a white ball and send messages to drive robot towards it.
 * 
 * @author Ron Johnson
 * @date 10/02/2019
 */
ros::ServiceClient client; /**< Define a global client that can request services */

// CONSTANTS
const uint DEFAULT_TARGET_COLOUR {255}; /**< Default target colour of object to chase, white is RGB {255,255,255} */ 
const float DEFAULT_ANG_Z {1.0};        /**< Default z angular rotation */
const float DEFAULT_LIN_X {1.0};        /**< Default x linear velocity */
const float LEFT  {0.3};                /**< Less than or equal to this % is defined as left pixel boundary */
const float RIGHT {1 - LEFT};           /**< Greater than or equal to this % is defined as right pixel boundary */
const uint R {0};                       /**< Define R(Red) index value */
const uint G {1};                       /**< Define G(Green) index value */
const uint B {2};                       /**< Define B(Blue) index value */

enum class DriveDir {Stop, Left, Fwd, Right}; /**< Which direction to drive the robot */
uint target_rgb[3] = {DEFAULT_TARGET_COLOUR, DEFAULT_TARGET_COLOUR, DEFAULT_TARGET_COLOUR};    /**< Init rgb target pixel colour to default */


/**
 * @brief Get an integer argument from the command line arg list.
 * 
 * @param arg_name
 * @param argc 
 * @param argv 
 * @return int 
 */
int get_arg_int(const char* arg_name, int argc, char** argv){
  int int_arg {};

  // Parse command line parameters
  for (auto i {1}; i < argc; i++) {  
    if (i + 1 != argc) {
      if ( strcmp(argv[i], arg_name) == 0) {
        std::string::size_type sz;   // alias of size_t
        int_arg = std::stoi(std::string(argv[i + 1]), &sz);
        break;
      }
    }
  }
  return int_arg;
}

/**
 * @brief Get a string argument from the command line arg list.
 * 
 * @param arg_name 
 * @param argc 
 * @param argv 
 * @return std::string 
 */
std::string get_arg_str(const char* arg_name, int argc, char** argv){
  std::string str_arg {};

  // Parse command line parameters
  for (auto i {1}; i < argc; i++) {  
    if (i + 1 != argc) {
      if ( strcmp(argv[i], arg_name) == 0) {
        str_arg = std::string(argv[i + 1]);
        break;
      }
    }
  }
  return str_arg;
}

/**
 * @brief Work out which direction to send the robot based on amount of target colour in left, mid or right part of image.
 * 
 * @param left 
 * @param mid 
 * @param right 
 * @return DriveDir 
 */
DriveDir getDriveDirection(uint left, uint mid, uint right){
  DriveDir dir {DriveDir::Stop};

  // Can we see any target colour pixels?
  if ( left + mid + right == 0 ){
    // Stop - dir set as default above
    ROS_INFO("ProcessImage:- Can't see target colour! Stopping robot.");
  } else {
    if ( left > right ){
      dir = DriveDir::Left;
    } else if ( right > left ){
      dir = DriveDir::Right;
    } else {
      dir = DriveDir::Fwd;
    }
  }

  return dir;
}

/**
 * @brief This function calls the command_robot service to drive the robot in the specified direction
 * 
 * @param lin_x 
 * @param ang_z 
 */
void drive_robot(float lin_x, float ang_z){
  ROS_INFO("ProcessImage:- Requesting DriveToTarget service - lin_x, ang_z = %0.2f, %0.2f", lin_x, ang_z);

  // Request DriveToTarget with required linear_x & angular_z target values
  ball_chaser::DriveToTarget srv {};
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  // Call the DriveToTarget service and pass the requested direction and speed
  if ( !client.call(srv) ) {
    ROS_ERROR("ProcessImage:- Failed to call the service drive_bot");
  }
}

/**
 * @brief This callback function continuously executes and reads the image data
 * 
 * @details Read image data 1 pixel at a time and check for the target colour. When we find 
 * a target colour we determine if the pixel is in the left part, right part or middle part of the 
 * camera view. This will then determine whether to drive forward, stop or turn.
 * 
 * @param img 
 */
void process_image_callback(const sensor_msgs::Image img){
    uint image_offset_left {static_cast<uint>(LEFT  * img.step)};   /**< Calc highest pixel column that defines the left part of the image */
    uint image_offset_right {static_cast<uint>(RIGHT * img.step)};  /**< Calc lowest pixel column that defines the right part of the image */

    uint left_count {0};
    uint mid_count {0};
    uint right_count {0};
    auto pixel_pos {0};
    auto img_size {img.height * img.step};

    // Loop all image data in groups of 3. Each 3 bytes being the RGB pixel colour
    for (auto i {2}; i <= img_size; i+=3){
      // Is this pixel the target colour?
      if ( img.data[i-2] == target_rgb[R] && img.data[i-1] == target_rgb[G] && img.data[i] == target_rgb[B]) {
        pixel_pos = i % img.step;

        // Now determine which region of the camera FOV the pixel is
        if ( pixel_pos <= image_offset_left ){
          left_count++;
        } else if ( pixel_pos >= image_offset_right ){
          right_count++;
        } else {
          mid_count++;
        }
      } // End if target colour pixel found
    }   // End for loop of all pixels

    // Get the direction to drive the robot
    DriveDir dir {getDriveDirection(left_count, mid_count, right_count)};

    // Calc how far to turn
    float lin_x {0.0};  // Default Stop
    float ang_z {0.0};  // Default No Turn
    switch ( dir ) {
      case DriveDir::Fwd: {
        lin_x = DEFAULT_LIN_X;
        break;
      }
      case DriveDir::Left: {
        ang_z = DEFAULT_ANG_Z;
        break;
      }
      case DriveDir::Right: {
        ang_z = -1 * DEFAULT_ANG_Z;
        break;
      }
      default: {
        // DriveDir::Stop  Default values already set
      }
    }

    // Drive the robot
    drive_robot(lin_x, ang_z);
}


/**
 * @brief Main Image driver
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv){
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle nh;

    // Get RGB colour arguments from command line
    target_rgb[R] = get_arg_int("-r", argc, argv);
    if ( target_rgb[R] < 0 || target_rgb[R] > 255 ) target_rgb[R] = DEFAULT_TARGET_COLOUR;
    target_rgb[G] = get_arg_int("-g", argc, argv);
    if ( target_rgb[G] < 0 || target_rgb[G] > 255 ) target_rgb[G] = DEFAULT_TARGET_COLOUR;
    target_rgb[B] = get_arg_int("-b", argc, argv);
    if ( target_rgb[B] < 0 || target_rgb[B] > 255 ) target_rgb[B] = DEFAULT_TARGET_COLOUR;
    ROS_INFO("ProcessImage:- Looking for object of target colour R%3d G%3d B%3d", target_rgb[R], target_rgb[G], target_rgb[B]);

    // Define a client service capable of requesting services from command_robot
    client = nh.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 =  nh.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}