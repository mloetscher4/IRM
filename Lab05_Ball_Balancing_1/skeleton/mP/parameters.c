#include "util.h"

// Allocate structs
parameter_struct bbs;
servo_struct servo;

/*Parameters related to robot hardware */
int load_parameters()
{
  // System Dimensions
  // All lengths in [mm]
  // Change this if you wan't to change plate height:
  bbs.plate_height = 120;
  // DON'T CHANGE THESE:
  bbs.R_plate_joint = 113.24;
  bbs.R_base_servo = 102.92;
  bbs.l1 = 42.0; // short arms
  bbs.l2 = 100.0;
  bbs.first_link_angle_bias = 4.6; // deg
  bbs.diameter_plate = 250.0;
  bbs.base_angles[0] = 0.5 * M_PI;          // rad
  bbs.base_angles[1] = (7.0 * M_PI) / 6.0;  // rad
  bbs.base_angles[2] = (11.0 * M_PI) / 6.0; // rad
  bbs.ball_radius = 20;

  // Camera Calibration Parameters:
  // **These are dummy parameters and need to be replaced with your calibration**:
  // When pictures are taken in PixyMon, their resolution varies. This factor
  // accounts for this scaling, such that the bbs can be copied directly
  bbs.calibration_image_scale = 3.54;
  bbs.focal_length = 460.6139;
  bbs.radial_distortion_coeff[0] = -0.2490;
  bbs.radial_distortion_coeff[1] = 0.0531;
  bbs.distortion_center[0] = 567.7938;
  bbs.distortion_center[1] = 361.2566;

  // Adjust these if you experience a constant offset in your coordinates the world frame
  bbs.t_wc[0] = -3;
  bbs.t_wc[1] = 7;
  bbs.t_wc[2] = -21;

  return 1;
}

/*Parameters related to the servo KST 05725MG. */
int load_servo()
{
  // Set angle bounds to avoid damage
  servo.min_angle = -50;
  servo.max_angle = 70;

  // Calibration of the mounting offset. Sum of bias should be as close to 0 as possible.
  // **These are dummy parameters and need to be replaced with your calibration**:

  servo.bias_A = 0;
  servo.bias_B = 9.936183-12.122352;
  servo.bias_C = 14.292622-12.122352;

  return 1;
}
