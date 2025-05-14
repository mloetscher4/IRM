#include "util.h"
#include "newton_raphson.h"
#include <math.h>

#define M_PI 3.14159265358979323846
#define PI_2 (M_PI / 2.0)
#define DEG2RAD(angle_deg) ((angle_deg) * M_PI / 180.0)
#define RAD2DEG(angle_rad) ((angle_rad) * 180.0 / M_PI)

int inverseKinematics(const double *plate_angles, double *servo_angles)
{
  // Load parameters R, L_1, L_2, P_z etc. from parameters file. Example: double R = bbs.R_plate_joint;
  // Then implement inverse kinematics similar to prelab

  double R = bbs.R_plate_joint;
  double L_1 = bbs.l1;
  double L_2 = bbs.l2;
  double P_Z = bbs.plate_height;

  // TODO return the angles alpha_A,B,C
  double delta_Z_A = R * sin(DEG2RAD(plate_angles[0]));
  double delta_Z_B = -0.5 * R * sin(DEG2RAD(plate_angles[0])) + sqrt(3)/2 * R * sin(DEG2RAD(plate_angles[1]));
  double delta_Z_C = -0.5 * R * sin(DEG2RAD(plate_angles[0])) - sqrt(3)/2 * R * sin(DEG2RAD(plate_angles[1]));  

  double beta_A = acos((pow(P_Z + delta_Z_A, 2.0) + pow(L_1, 2.0) - pow(L_2, 2.0)) / (2 * L_1 * (P_Z + delta_Z_A)));
  double alpha_A = PI_2 - beta_A;
  double beta_B = acos((pow(P_Z + delta_Z_B, 2.0) + pow(L_1, 2.0) - pow(L_2, 2.0)) / (2 * L_1 * (P_Z + delta_Z_B)));
  double alpha_B = PI_2 - beta_B;
  double beta_C = acos((pow(P_Z + delta_Z_C, 2.0) + pow(L_1, 2.0) - pow(L_2, 2.0)) / (2 * L_1 * (P_Z + delta_Z_C)));
  double alpha_C = PI_2 - beta_C;

  servo_angles[0] = RAD2DEG(alpha_A);
  servo_angles[1] = RAD2DEG(alpha_B);
  servo_angles[2] = RAD2DEG(alpha_C);

  
  if(fabs(plate_angles[0]) > 45 || fabs(plate_angles[1]) > 45)
  {
    printf("ERROR: Plate angles out of bounds.\n");
    return -1;
  }

  return 0;
}

int project2worldFrame(const int x_in, const int y_in, double *x_out, double *y_out)
{
// implement the code to project the coordinates in the image frame to the world frame
  // make sure to multiply the raw pixy2 coordinates with the scaling factor (ratio between
  // image fed to python for calibration and pixy2 resolution): bbs.calibration_image_scale.

  // 1. scale with calibration factor
  double u_pix = x_in * bbs.calibration_image_scale;
  double v_pix = y_in * bbs.calibration_image_scale;
  
  // 2. normalize before undistorting (u_bar = K^-1 * u)
  double u_bar = (u_pix - bbs.distortion_center[0]) / bbs.focal_length;
  double v_bar = (v_pix - bbs.distortion_center[1]) / bbs.focal_length;

  // 3. undistort images with netwon raphson (r_d is distorted, r is undistorted)
  double r_d = sqrt(pow(u_bar, 2) + pow(v_bar, 2));
  double r = newtonRaphson(r_d, bbs.radial_distortion_coeff[0], bbs.radial_distortion_coeff[1]);
  
  // 4. undistort in image frame
  double u_bar_undist = (r / r_d) * u_bar;
  double v_bar_undist = (r / r_d) * v_bar;

  // 5. Unnormalize in image frame 
  double u_undist = u_bar_undist * bbs.focal_length;
  double v_undist = v_bar_undist * bbs.focal_length;
 
  // 6. Compute lambda
  double lambda = bbs.plate_height + bbs.ball_radius - bbs.t_wc[2];

  // 7. Solve for x_world and y_world
  double x_cam = lambda * u_undist;
  double y_cam = lambda * v_undist;

  x_cam = x_cam/(-bbs.focal_length);
  y_cam = y_cam/(-bbs.focal_length);

  double x_world = x_cam + bbs.t_wc[0];
  double y_world = y_cam + bbs.t_wc[1];

  *x_out = x_world;
  *y_out = y_world;

  return 0;
};

double discreteDerivative(const double dt, const double *x)
{
  // TODO: Implement a discrete derivative function

  /* ********************* */
  /* Insert your Code here */
  /* ********************* */

  double vel;
  // single precision finite difference approach
  vel = (x[0] - x[1]) / dt;

  // more accurate approach backwards finit difference with precision of 3
  //vel = ((11/6) * x[0] - 3 * x[1] + (3/2) * x[2] - (1/3) * x[3]) / dt;
  return vel;
};

double movingAverage(const int n, const double *x)
{
  // TODO: Implement a moving average function
  // TODO: not sure if this is how we have to do it, as this function now only computes one average value for the first n entries in x
  double sum = 0.0;
  
  for(int i = 0; i<n; i++){
    sum += x[i];
  }

  return sum / n;
};

double butterWorth(const double *x)
{
  // TODO: Implement this if you like bonus points (not required to reach max points)
  //double b[3] = {0.0134, 0.0267, 0.0134};
  //double a[3] = {1.0000, -1.6475, 0.7009};
  
  int n = 5;
  double b[5] = {0.0466, 0.1863,    0.2795,    0.1863,    0.0466};
  double a[5] = {1.0000,   -0.7821,    0.6800,   -0.1827,    0.0301};

  double den = 0.0; 
  double num = 0.0; 

  for(int i = 0; i<n; i++){
    den += a[i]*x[i]; 
    num += b[i]*x[i];
  }

  return num/den;
};

int stepResponse(const double current_time, double *x_ref, double *y_ref,
                 double *vx_ref, double *vy_ref)
{

  double step_start = 5;
  double step_distance = 80;
  if (current_time < step_start)
  {
    *x_ref = 0;
  }
  else
  {
    *x_ref = step_distance;
  }

  *y_ref = 0;
  *vx_ref = 0;
  *vy_ref = 0;

  return 0;
};

int circularTrajectory(const double current_time, double *x_ref, double *y_ref,
                       double *vx_ref, double *vy_ref)
{

  double traj_start = 3;
  double num_of_traj = 5;
  double period = 4; // seconds
  double R = 75;     // radius

  // TODO: Implement the circular trajectory function.
  //  Hint: Use the equations for parametrizing a cirlce (and its derivative)
  if(current_time < traj_start){
    *x_ref = 0; 
    *y_ref = 0; 
    *vx_ref = 0; 
    *vy_ref = 0; 
  }
  else{
    double phase = (current_time-traj_start)*2*M_PI/period; 
    *x_ref = R*sin(phase); 
    *y_ref = R*cos(phase); 
    *vx_ref = R*cos(phase)*2*M_PI/period;
    *vy_ref = -R*sin(phase)*2*M_PI/period;
  } 

  return 0;
};

// Don't change any of the below functions.

int initBallBalancingRobot(int fd)
{
  // don't change this function.
  printf("\e[1;1H\e[2J"); // Clear screen
  printf("#######################\n");
  printf("Hardware Initialization\n");
  printf("#######################\n");

  // Let feather reboot
  usleep(200);

  // Check pixy readings
  int pixy_return, flag, x, y;
  pixy_return = readFromPixy(fd, &flag, &x, &y);
  while (!pixy_return)
  {
    // retry until connection established
    pixy_return = readFromPixy(fd, &flag, &x, &y);
  }

  printf("%s, x = %d, y = %d \n", "INIT: Pixy Coordinates received", x, y);

  // Do some inverse kinematics
  double position[] = {0, 0, 130};
  double plate_angles[] = {0, 0};
  double servo_angles[] = {0, 0, 0};
  inverseKinematics(plate_angles, servo_angles);
  printf("INIT: Inverse Kinematics initialized\n");
  printf("INIT: Finished\n");
  // printf("INIT: A: %.2f, B: %.2f, C: %.2f
  // \n",servo_angles[0],servo_angles[1],servo_angles[2]);

  tcflush(fd, TCIFLUSH);

  // Send motor commands
  // servoCommand(fd,servo_angles);

  return 1;
}

/* Sends servo angles to serial port */
int servoCommand(int fd, double *servo_angles)
{
  // check serial
  int writeval;

  // assign values
  double angleA = servo_angles[0] + servo.bias_A;
  double angleB = servo_angles[1] + servo.bias_B;
  double angleC = servo_angles[2] + servo.bias_C;

  int min = servo.min_angle;
  int max = servo.max_angle;

  // check if values are valid
  int condition = (angleA < max && angleA > min) &&
                  (angleB < max && angleB > min) &&
                  (angleC < max && angleC > min);

  if (condition != 1)
  {
    printf("ERROR: Servo angles out of bounds.\n");
    return -1;
  }

  // assemble command
  char command[50];
  sprintf(command, "C %.2f %.2f %.2f\n", angleA, angleB, angleC);

  // Flush serial port output
  tcflush(fd, TCOFLUSH);
  // send command
  writeval = write(fd, command, strlen(command));
  usleep(1200); // wait for write function to complete

  return 0;
}

/* Reads pixel coordinates from Pixycam. Also returns a flag whether an object
 * was detected or not */
int readFromPixy(int fd, int *flag, int *x, int *y)
{
  char buff[20];
  const char command[] = "P\n";
  int writeval;
  char *token;
  const char delim[] = " ";

  // Flush serial port input
  tcflush(fd, TCIFLUSH);
  tcflush(fd, TCOFLUSH);

  // Write command to pixy
  writeval = serialport_write(fd, command);
  usleep(10 * 1000);

  // Read until until no more bytes available
  // If not data is availabe, retry until success
  int readval = 0;
  while (readval != 1)
  {
    readval = serialport_read_until(fd, buff, sizeof(buff), '\n', 100);
  }

  // printf("readFromPixy: after read \n");
  // printf("writeval = %d, readval = %d", writeval,readval);

  // Catch read write errors
  if (!readval)
  {
    // printf("SERIAl READ FAILED with %d \n",readval);
    return -1;
  }

  // Add terminating 0 to make string
  buff[sizeof(buff) - 1] = 0;

  // extract values using strtok
  token = strtok(buff, delim);

  // Verify initial character
  if (token[0] != 'A')
  {
    // printf("SERIAL HEADER ERROR: %.20s\n",buff);
    return -1;
  }

  token = strtok(NULL, delim);
  *flag = atoi(token);
  token = strtok(NULL, delim);
  token = strtok(NULL, delim);
  *x = atoi(token);
  token = strtok(NULL, delim);
  token = strtok(NULL, delim);
  *y = atoi(token);

  return 1;
}

long getMicroseconds()
{
  struct timeval the_time;
  long microseconds;
  gettimeofday(&the_time, NULL);
  microseconds = the_time.tv_sec * 1000000 + the_time.tv_usec / 1000 + the_time.tv_usec;
  return microseconds;
}

int pushBack(const double x_new, double *x_array, int array_size)
{
  for (int i = array_size - 1; i > 0; i = i - 1)
  {
    x_array[i] = x_array[i - 1];
  }

  x_array[0] = x_new;
  return 1;
};

int startLogging(FILE *fp, int task_selection, double k_p, double k_d,
                 double k_i, int n_pos, int n_vel)
{
  fprintf(fp, "#Task Selector: %d\n", task_selection);
  fprintf(fp, "#PID Parameters: kp: %.2f, kd: %.2f, ki: %.2f\n", k_p, k_d, k_i);
  fprintf(fp, "#n_pos: %d\n", n_pos);
  fprintf(fp, "#n_vel: %d\n", n_vel);
  fprintf(fp,
          "t x_ref y_ref vx_ref vy_ref x_raw y_raw x y vx_raw vy_raw vx vy\n");
  fclose(fp);

  return 0;
}

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
           double y_integrator)
{

  printf("\e[1;1H\e[2J");
  printf("##################################\n");
  printf("############# PID LOOP ###########\n");
  printf("##################################\n");
  printf("Timing\n");
  printf("Elapsed Time: %.2f s\n", current_time);
  printf("Step Time: %.4f s\n", dt);
  printf("Frequency: %.1f Hz\n\n", 1 / dt);
  printf("PID Parameters\n");
  printf("k_p: %.2f, k_d: %.2f, k_i: %.2f\n\n", k_p, k_d, k_i);
  printf("ReferencePosition\n");
  printf("x_ref: %.f \t y_ref: %.f\n\n", x_ref, y_ref);
  printf("Position\n");
  printf("x: %.f \t y: %.f\n\n", x, y);
  printf("Reference Velocity\n");
  printf("vx_ref: %.f \t vy_ref: %.f\n\n", vx_ref, vy_ref);
  printf("Velocity\n");
  printf("vx: %.f \t vy: %.f\n\n", vx, vy);
  printf("Plate Angles\n");
  printf("pitch: %.f deg, roll: %.f deg \n\n", plate_angles[1], plate_angles[0]);
  printf("Servo Angles\n");
  printf("A: %.f deg, B: %.f deg, C: %.f deg\n\n", servo_angles[0], servo_angles[1], servo_angles[2]);
  printf("Integrators\n");
  printf("x_integ: %.2f, y_integ: %.2f\n", x_integrator, y_integrator);

  fprintf(
      fp,
      "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
      current_time, x_ref, y_ref, vx_ref, vy_ref, x_raw, y_raw, x, y, vx_raw,
      vy_raw, vx, vy);
  fclose(fp);

  return 0;
};
