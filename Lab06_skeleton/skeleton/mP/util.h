#include "feather_serial.h"
#include "invkin.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#define M_PI 3.14159265358979323846
/*
 * Parameter structs: two new types are defined which
 * will be used to access calibration parameters. The
 * "extern" keyword tells the compiler that the variable
 * has been declared elsewhere, and that this variable
 * should be linked. This way, the structs bbs and servo
 * are globally available and do not need to be entered
 * as arguments to each function.
 * In this case, the variables are declared in the file
 * parameters.c. This file also includes definitions for
 * functions which load the parameter values. The
 * declarations for these functions are here in util.h,
 * since they will be called by main.c
 */

typedef struct
{
  double plate_height;
  double R_plate_joint;
  double R_base_servo;
  double l1;
  double l2;
  double first_link_angle_bias;
  double diameter_plate;
  double base_angles[3];
  double ball_radius;
  double focal_length;
  double radial_distortion_coeff[2];
  double distortion_center[2];
  double calibration_image_scale;
  double t_wc[3];
  int window_size;
} parameter_struct;
extern parameter_struct bbs;

typedef struct
{
  int min_angle;
  int max_angle;
  double bias_A;
  double bias_B;
  double bias_C;
} servo_struct;
extern servo_struct servo;

/*Loads constant system parameters*/
int load_parameters();

/*Loads parameters related to the servomotors used */
int load_servo();

int inverseKinematics(const double *plate_angles, double *servo_angles);

int project2worldFrame(const int x_in, const int y_in, double *x_out, double *y_out);

int updateStates(const double dt, double *pos_meas, double *pos,
                 double *vel_raw, double *vel);

double discreteDerivative(const double dt, const double *x);

/*moving average filter
 n: filter length
 x: array to be filtered; at least of length n.
 return: filtered point.
*/
double movingAverage(const int n, const double *x);

double butterWorth(const double *x, const double *y);

/*step response
 current_time: current timestep (step should execute after this reaches a certain treshhold, e.g. 5 sec)
 x_ref: variable to store x-position output of circularTrajectory function
 y_ref: variable to store y-position output of circularTrajectory function
 vx_ref: variable to store x-velocity output of circularTrajectory function
 vy_ref: variable to store y-velocity output of circularTrajectory function
 return: 0
 */
int stepResponse(const double current_time, double *x_ref, double *y_ref,
                 double *vx_ref, double *vy_ref);

/*circular trajectrory
current_time: timestep in trajectory (trajectory should execute after this reaches a certain treshhold, e.g. 3 sec)
x_ref: variable to store x-position output of circularTrajectory function
y_ref: variable to store y-position output of circularTrajectory function
vx_ref: variable to store x-velocity output of circularTrajectory function
vy_ref: variable to store y-velocity output of circularTrajectory function
return: 0
 */
int circularTrajectory(const double current_time, double *x_ref, double *y_ref,
                       double *vx_ref, double *vy_ref);

/*initializes and tests system */
int initBallBalancingRobot(int fd);

int servoCommand(int fd, double *servo_angles);

int readFromPixy(int fd, int *flag, int *x, int *y);

long getMilliseconds();

long getMicroseconds();

int pushBack(const double x_new, double *x_array, int array_size);

int startLogging(FILE *fp, int task_selection, double k_p, double k_d,
                 double k_i, int n_pos, int n_vel);

int logger(FILE *fp,
           long end_time,
           double current_time,
           double dt,
           double k_p,
           double k_d,
           double k_i,
           double x_ref,
           double y_ref,
           double vx_ref,
           double vy_ref,
           double x_raw,
           double y_raw,
           double x,
           double y,
           double vx_raw,
           double vy_raw,
           double vx,
           double vy,
           double *plate_angles,
           double *servo_angles,
           double x_integrator,
           double y_integrator);
