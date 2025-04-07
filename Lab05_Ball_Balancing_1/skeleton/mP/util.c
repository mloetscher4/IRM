#include "util.h"
#include "newton_raphson.h"
#include <math.h>

#define M_PI 3.14159265358979323846
#define PI_2 (M_PI / 2.0)
#define DEG2RAD(angle_deg) ((angle_deg) * M_PI / 180.0)
#define RAD2DEG(angle_rad) ((angle_rad) * 180.0 / M_PI)

int initBallBalancingRobot(int fd)
{

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

  // Do some inverse kinematics to check for errors
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

int inverseKinematics(const double *plate_angles, double *servo_angles)
{
  // Load parameters R, L_1, L_2, P_z etc. from parameters file. Example: double R = bbs.R_plate_joint;
  // Then implement inverse kinematics similar to prelab

  /* ********************* */
  /* Insert your Code here */
  /* ********************* */
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

  return 0; // if ok
};

int project2worldFrame(const int x_in, const int y_in, double *x_out, double *y_out)
{

  // implement the code to project the coordinates in the image frame to the world frame
  // make sure to multiply the raw pixy2 coordinates with the scaling factor (ratio between
  // image fed to python for calibration and pixy2 resolution): bbs.calibration_image_scale.

  /* ********************* */
  /* Insert your Code here */
  /* ********************* */

  return 0;
};

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
