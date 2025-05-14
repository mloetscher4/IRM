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
    int repeat = 1;     

    while(repeat){
    printf("Please enter requested plate angles: \n");
    scanf("%lf",plate_angles);
    scanf("%lf",plate_angles+1); 
    
    inverseKinematics(plate_angles,servo_angles);

    // Limits on the servo angles are already set in the servo command function: 
    servoCommand(fd,servo_angles);

    printf("Euler (Plate) Angles: %f %f \n", plate_angles[0],plate_angles[1]); 
    printf("Servo Angles: %f %f %f \n", servo_angles[0],servo_angles[1],servo_angles[2]); 
    printf("\n Enter 1 to repeat or 0 to stop: ");
    
    scanf("%d", &repeat);

    printf("\n \n");
    }
  }

  //////////////////////////////
  /////////// Task 2 ///////////
  //////////////////////////////
  /*Test camera calibration*/
  if (task_selection == 2)
  {
    // initalize variables:
    int x_pxy;
    int y_pxy;
    double x_wrld;
    double y_wrld;
    int successful_read;
    int ball_detected;
    int successful_conversion;
    int repeat = 1; 

    while(repeat){
    successful_read = readFromPixy(fd, &ball_detected, &x_pxy, &y_pxy);

    if (successful_read == -1)
    {
      printf("Error when reading from serial port");
    }

    if(ball_detected == 1)
    {
      successful_conversion = project2worldFrame(x_pxy, y_pxy, &x_wrld, &y_wrld);
      printf("\n");
      printf("Distorted Pixel Coordinates\n");
      printf("x: %i y: %i \n \n", x_pxy, y_pxy);
      printf("World frame coordinates\n");
      printf("x: %f y: %f \n", x_wrld, y_wrld);
    }
    else if (ball_detected == 0)
    {
      printf("No ball was detected");
    }
    printf("\n Enter 1 to repeat or 0 to stop: ");

    scanf("%d", &repeat);

    printf("\n \n");
    }
    

    return 0;
  }

  //////////////////////////////
  /////// Task 4/5/6 ///////////
  //////////////////////////////

  if((task_selection == 4) || (task_selection == 5) || (task_selection == 6))
  {

    // TODO: Initialize default PID parameters
    double k_p = 0.087;
    double k_d = 0.04;
    double k_i = 0.01;

    // TODO: Intialize filter window size
    int n_pos = 10;
    int n_vel = 10;

    // TODO: Ask for user input to change PID parameters
    /* ********************* */
    /* Insert your Code here */
    /* ********************* */
    printf("Please enter the PID params Kp, Kd, Ki if you would like to change them!\n");
    scanf("%lf", &k_p);
    scanf("%lf", &k_d);
    scanf("%lf", &k_i);


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
    int x_pixy;
    int y_pixy;

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
    long counter = 0;

    while (1)
    {
      
      /* ********************* */
      /* Insert your Code here */
      /* ********************* */
      
      // TODO: Get current sampling time dt
      // TODO: Get the coordinates of the ball in the Pixy Camera frame (Use a function in util.c)
      readFromPixy(fd, &flag, &x_pixy, &y_pixy);

      // If the ball is detected, enter if-bracket
      if (flag)
      {
        // TODO: Use camera calibration form Lab05
        pushBack(0, x_raw, buf_size);
        pushBack(0, y_raw, buf_size);

        project2worldFrame(x_pixy, y_pixy, x_raw, y_raw);

        // TODO: Place measurements in buffer array
        // Hint: There is a function called pushBack
        //  in util.h that you can use here.

        // TODO: Apply filter to position coordinates
        pushBack(movingAverage(n_pos, x_raw), x, buf_size);
        pushBack(movingAverage(n_pos, y_raw), y, buf_size);

        // TODO: Compute velocity based on the filtered position signal
        // TODO: Place velocity in buffer array (use pushBack function)
        pushBack(discreteDerivative(dt, x), vx_raw, buf_size);
        pushBack(discreteDerivative(dt, y), vy_raw, buf_size);
        
        // TODO: Apply filter to velocity
        //pushBack(movingAverage(n_vel, vx_raw), vx, buf_size);
        //pushBack(movingAverage(n_vel, vy_raw), vy, buf_size);
        double vx_filtered = butterWorth(vx_raw, vx);
        double vy_filtered = butterWorth(vy_raw, vy);
        pushBack(vx_filtered, vx, buf_size);
        pushBack(vy_filtered, vy, buf_size);
        
        // TODO: Set reference depending on task
        switch (task_selection)
        {
          case 4: /*TODO: Postlab Q4 centering task */
            x_ref = 0.0;
            y_ref = 0.0;
            vx_ref = 0.0;
            vy_ref = 0.0;
            break;
          case 5: /*TODO: Postlab Q5 step response reference  --> use function in util.h */
            stepResponse(current_time, &x_ref, &y_ref, &vx_ref, &vy_ref);
          break;
          case 6: /*TODO: Postlab Q6 circular trajectory reference --> implement & use function in util.h */
            circularTrajectory(current_time, &x_ref, &y_ref, &vx_ref, &vy_ref);
          break;
        }
        
        // TODO: Update Integrator after an initial delay
        // Hint: Wait 0.5s before starting to update integrator
        if(current_time > 0.5){
          x_integ += (x[0] - x_ref) * dt;
          y_integ += (y[0] - y_ref) * dt;
        }

        // TODO: Compute PID (remember, PID output is the plate angles)
        // TODO: Define Plate angles from PID output (watch out for correct sign)
        plate_angles[1] = - k_i * x_integ + k_p * (x_ref - x[0]) + k_d * (vx_ref - vx[0]);
        plate_angles[0] = -(- k_i * y_integ + k_p * (y_ref - y[0]) + k_d * (vy_ref - vy[0]));
        
        // TODO: Compute servo angles and send command
        inverseKinematics(plate_angles, servo_angles);
        servoCommand(fd, servo_angles);
        
        // calculating dt based on the time passed in the while loop iter
        // doing this at the end to avoid first iteration issues
        dt = ((getMicroseconds() - t0) * 1.0e-6 - current_time);
        current_time = (getMicroseconds() - t0) * 1.0e-6;
        
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
