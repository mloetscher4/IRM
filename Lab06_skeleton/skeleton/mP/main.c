#include "util.h"
#include <stdio.h>

int main()
{
  //////////////////////////////
  /////// Initialization ///////
  //////////////////////////////

  // Time information
  time_t rawtime;
  struct tm *info;
  char buffer[80];
  time(&rawtime);
  info = localtime(&rawtime);
  printf("Formatted date & time : |%s|\n", buffer);

  // Print Welcome Message
  printf("\e[1;1H\e[2J"); // Clear screen
  printf("#####################\n");
  printf("Ball and Plate System\n");
  printf("#####################\n");
  printf("|%s|\n", buffer);
  printf("\n");
  printf("Opening serial port...\n");

  // Initialize the serial port
  const char *port = "/dev/ttyUSB0";
  int fd = serialport_init(port, 115200);
  if (fd == -1)
  {
    printf("Could not open the port.\n");
    printf(" - Is the Arduino IDE terminal opened?\n");
    printf(" - Is the device connected to the VM?\n");
    return -1;
  }

  // Initialize robot and check
  // if messages are received
  initBallBalancingRobot(fd);

  // Make sure that serial port is relaxed
  usleep(20 * 1000);

  // Parameter loading functions
  load_parameters();
  load_servo();

  //////////////////////////////
  //////// Task Selection //////
  //////////////////////////////
  int task_selection = 0;
  printf("Select Task: ");
  scanf("%d", &task_selection);

  //////////////////////////////
  /////////// Task 1 ///////////
  //////////////////////////////

  if (task_selection == 1)
  {
    /* Test inverse kinematics via
     terminal */

    // initalize variables:
    double plate_angles[] = {0, 0};
    double servo_angles[] = {0, 0, 0};

    /* ********************* */
    /* Insert your Code here (From Lab 05)*/
    /* ********************* */
  }

  //////////////////////////////
  /////////// Task 2 ///////////
  //////////////////////////////
  /*Test camera calibration*/
  if (task_selection == 2)
  {
    // initalize variables:

    /* ********************* */
    /* Insert your Code here (From Lab 05) */
    /* ********************* */
  }

  //////////////////////////////
  /////// Task 4/5/6 ///////////
  //////////////////////////////

  if ((task_selection == 4) || (task_selection == 5) || (task_selection == 6))
  {

    // TODO: Initialize default PID parameters
    double k_p = 0;
    double k_d = 0;
    double k_i = 0;

    // TODO: Intialize filter window size
    int n_pos = 0;
    int n_vel = 0;

    // TODO: Ask for user input to change PID parameters
    /* ********************* */
    /* Insert your Code here */
    /* ********************* */

    // Variables for Pixy2
    int flag = 0;      // flag that detects if the pixy cam can detect a ball
    int x_px = 0;      // raw x coordinate read by pixy cam
    int y_px = 0;      // raw y coordinate read by pixy cam
    double x_cal = 0;  // calibrated x coordinate in plate frame and mm
    double y_cal = 0;  // calibrated y coordinate in plate frame and mm
    double x_filt = 0; // filtered x coordinate in plate frame and mm
    double y_filt = 0; // filtered y coordinate in plate frame and mm
    double vel_x = 0;  // x velocity calculated from filtered position
    double vel_y = 0;  // y velocity calculated from filtered position

    // read pixy a couple of times to clear buffer
    for (int i = 0; i < 20; i++)
    {
      readFromPixy(fd, &flag, &x_px, &y_px);
    }

    // create buffer arrays for filtered variables
    // [0] is always the current element
    // make sure buf_size is bigger than filter windows
    int buf_size = 50;
    double x_raw[buf_size]; // calibrated, unfiltered
    double y_raw[buf_size];
    double vx_raw[buf_size]; // 1st order derivative
    double vy_raw[buf_size];
    double x[buf_size]; // filtered position
    double y[buf_size];
    double vx[buf_size]; // filtered velocity
    double vy[buf_size];

    // initialize buffer arrays to zero
    for (int i = 0; i < buf_size; i++)
    {
      x_raw[i] = 0;
      y_raw[i] = 0;
      vx_raw[i] = 0;
      vy_raw[i] = 0;
      x[i] = 0;
      y[i] = 0;
      vx[i] = 0;
      vy[i] = 0;
    }

    // initialize angles
    double servo_angles[] = {0, 0, 0};
    double plate_angles[] = {0, 0};

    // reference variables for control
    // are being set in reference functions
    double x_ref, y_ref, vx_ref, vy_ref;

    // pid variables
    double x_integ = 0;
    double y_integ = 0;
    double u_x = 0;
    double u_y = 0;

    // Logfile with datetime as filename
    char datetime[80];
    strftime(datetime, 80, "%Y-%m-%d_%H-%M-%S_pid_log.txt", info);
    FILE *fp = fopen(datetime, "w+");
    startLogging(fp, task_selection, k_p, k_d, k_i, n_pos, n_vel);

    // Timing variables
    //  TODO: Measure the current sampling time dt (in seconds, for the derivative): It is the time it takes to run the previous loop iteration.
    //  Hint: use getMicroseconds() and don't forget to convert to seconds.
    long start = 0;
    long end = 16;
    long t0 = getMicroseconds(); // get starting time of the loop
    double dt = 0.016;           // variable for timing
    double current_time = 0;

    while (1)
    {

      /* ********************* */
      /* Insert your Code here */
      /* ********************* */

      // TODO: Get current sampling time dt

      // TODO: Get the coordinates of the ball in the Pixy Camera frame (Use a function in util.c)

      // If the ball is detected, enter if-bracket
      if (flag)
      {
        // TODO: Use camera calibration form Lab05

        // TODO: Place measurements in buffer array
        // Hint: There is a function called pushBack
        //  in util.h that you can use here.

        // TODO: Apply filter to position coordinates

        // TODO: Compute velocity based on the filtered position signal

        // TODO: Place velocity in buffer array (use pushBack function)

        // TODO: Apply filter to velocity

        // TODO: Set reference depending on task
        switch (task_selection)
        {
        case 4: /*TODO: Postlab Q4 centering task */
          break;
        case 5: /*TODO: Postlab Q5 step response reference  --> use function in util.h */
          break;
        case 6: /*TODO: Postlab Q6 circular trajectory reference --> implement & use function in util.h */
          break;
        }

        // TODO: Update Integrator after an initial delay
        // Hint: Wait 0.5s before starting to update integrator

        // TODO: Compute PID (remember, PID output is the plate angles)

        // TODO: Define Plate angles from PID output (watch out for correct sign)

        // TODO: Compute servo angles and send command

        // Open logging file and log everything to textfile
        fp = fopen(datetime, "a");
        logger(fp, end, current_time, dt, k_p, k_d, k_i, x_ref, y_ref, vx_ref,
               vy_ref, x_raw[0], y_raw[0], x[0], y[0], vx_raw[0], vy_raw[0],
               vx[0], vy[0], plate_angles, servo_angles, x_integ, y_integ);
      }
    }
  }

  return 0;
}
