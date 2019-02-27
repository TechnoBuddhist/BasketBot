#include <iostream>
#include <string>
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

/**
 * @brief Search for a white ball and send messages to drive robot towards it.
 */

ros::ServiceClient rosServiceClient;


const uint DEFAULT_TARGET_COLOUR {255}; /**< Default target colour of object to chase, white is RGB {255,255,255} */ 
uint target_rgb[3] = {DEFAULT_TARGET_COLOUR, DEFAULT_TARGET_COLOUR, DEFAULT_TARGET_COLOUR}; /**< Init rgb target pixel colour to default */
const float DEFAULT_ANGULAR_Z {1.0};
const float DEFAULT_LINEAR_X {1.0};
const float LEFT_BOUNDARY  {0.3};               /**< Less than or equal to this % is defined as left pixel boundary */
const float RIGHT_BOUNDARY {1 - LEFT_BOUNDARY}; /**< Greater than or equal to this % is defined as right pixel boundary */
const uint R {0};
const uint G {1};
const uint B {2};
enum class DriveDirection {Stop, Left, Fwd, Right};


/**
 * @brief Get an integer argument from the command line argument list.
 */
int get_integer_argument(const char* arg_name, int argc, char** argv){
  int command_line_argument {};

  // Parse command line parameters
  for (auto i {1}; i < argc; i++) {  
    if (i + 1 != argc) {
      if ( strcmp(argv[i], arg_name) == 0) {
        std::string::size_type sz;   // alias of size_t
        command_line_argument = std::stoi(std::string(argv[i + 1]), &sz);
        break;
      }
    }
  }
  return command_line_argument;
}

/**
 * @brief Get a string argument from the command line argument list.
 */
std::string get_string_argument(const char* arg_name, int argc, char** argv){
  std::string command_line_argument {};

  // Parse command line parameters
  for (auto i {1}; i < argc; i++) {  
    if (i + 1 != argc) {
      if ( strcmp(argv[i], arg_name) == 0) {
        command_line_argument = std::string(argv[i + 1]);
        break;
      }
    }
  }
  return command_line_argument;
}

/**
 * @brief Work out which direction to send the robot based on amount of target colour in left, mid or right part of image.
 */
DriveDirection get_drive_direction(uint left_pixel_count, uint mid_pixel_count, uint right_pixel_count){
  DriveDirection direction {DriveDirection::Stop};

  // Can we see any target colour pixels?
  if ( left_pixel_count + mid_pixel_count + right_pixel_count == 0 ){
    // Stop - direction set as default above
    ROS_INFO("ProcessImage:- Can't see target colour! Stopping robot.");
  } else {
    if ( left_pixel_count> right_pixel_count ){
      direction = DriveDirection::Left;
    } else if ( right_pixel_count > left_pixel_count ){
      direction = DriveDirection::Right;
    } else {
      direction = DriveDirection::Fwd;
    }
  }

  return direction;
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
  ball_chaser::DriveToTarget rosDriveToService {};
  rosDriveToService.request.linear_x = lin_x;
  rosDriveToService.request.angular_z = ang_z;

  if ( !rosServiceClient.call(rosDriveToService) ) {
    ROS_ERROR("ProcessImage:- Failed to call the service drive_bot");
  }
}

/**
 * @brief This callback function continuously executes and reads the image data
 * 
 * @details Read image data 1 pixel at a time and check for the target colour. When we find 
 * a target colour we determine if the pixel is in the left part, right part or middle part of the 
 * camera view. This will then determine whether to drive forward, stop or turn.
 */
void process_image_callback(const sensor_msgs::Image img){
    // Calculate the pixel columns that define the left and right part of the image
    uint image_offset_left {static_cast<uint>(LEFT_BOUNDARY  * img.step)};
    uint image_offset_right {static_cast<uint>(RIGHT_BOUNDARY * img.step)};

    uint left_pixel_count {0};
    uint mid_pixel_count {0};
    uint right_pixel_count {0};
    auto pixel_position {0};
    auto img_size {img.height * img.step};

    // Loop all image data in groups of 3. Each 3 bytes being the RGB pixel colour
    for (auto i {2}; i <= img_size; i+=3){
      // Is this pixel the target colour?
      if ( img.data[i-2] == target_rgb[R] && img.data[i-1] == target_rgb[G] && img.data[i] == target_rgb[B]) {
        pixel_position = i % img.step;

        // Now determine which region of the camera FOV the pixel is
        if ( pixel_position <= image_offset_left ){
          left_pixel_count++;
        } else if ( pixel_position >= image_offset_right ){
          right_pixel_count++;
        } else {
          mid_pixel_count++;
        }
      } // End if target colour pixel found
    }   // End for loop of all pixels

    DriveDirection dir {get_drive_direction(left_pixel_count, mid_pixel_count, right_pixel_count)};

    // Set the linear velocity and angular rotation
    float lin_x {0.0};  // Default Stop
    float ang_z {0.0};  // Default No Turn
    switch ( dir ) {
      case DriveDirection::Fwd: {
        lin_x = DEFAULT_LINEAR_X;
        break;
      }
      case DriveDirection::Left: {
        ang_z = DEFAULT_ANGULAR_Z;
        break;
      }
      case DriveDirection::Right: {
        ang_z = -1 * DEFAULT_ANGULAR_Z;
        break;
      }
      default: {
        // DriveDirection::Stop  Default values already set
      }
    }

    drive_robot(lin_x, ang_z);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "process_image");
    ros::NodeHandle rosNodeHandle;

    // Get RGB colour arguments from command line
    target_rgb[R] = get_integer_argument("-r", argc, argv);
    if ( target_rgb[R] < 0 || target_rgb[R] > 255 ) target_rgb[R] = DEFAULT_TARGET_COLOUR;
    target_rgb[G] = get_integer_argument("-g", argc, argv);
    if ( target_rgb[G] < 0 || target_rgb[G] > 255 ) target_rgb[G] = DEFAULT_TARGET_COLOUR;
    target_rgb[B] = get_integer_argument("-b", argc, argv);
    if ( target_rgb[B] < 0 || target_rgb[B] > 255 ) target_rgb[B] = DEFAULT_TARGET_COLOUR;
    ROS_INFO("ProcessImage:- Looking for object of target colour R%3d G%3d B%3d", target_rgb[R], target_rgb[G], target_rgb[B]);

    rosServiceClient = rosNodeHandle.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    ros::Subscriber rosSubscriber =  rosNodeHandle.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    ros::spin();

    return 0;
}
