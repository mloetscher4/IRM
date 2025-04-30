#include "util.h"
#include <stdio.h>

int main()
{
  //////////////////////////////
  /////// Initialization ///////
  //////////////////////////////

  // Print Welcome Message
  printf("\e[1;1H\e[2J"); // Clear screen
  printf("#####################\n");
  printf("Ball and Plate System\n");
  printf("#####################\n");
  printf("\n");
  printf("Opening serial port...\n");

  // Initialize the serial port
  const char *port = "/dev/ttyUSB0"; // vm: "/dev/ttyUSB0", mac: "/dev/cu.SLAB_USBtoUART"
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
  /*Test projection from the image frame to the world frame*/
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

    successful_read = readFromPixy(fd, &ball_detected, &x_pxy, &y_pxy);

    if (successful_read == 1)
    {
      printf("Error when reading from serial port");
    }
    
    if(ball_detected == 1)
    {
      successful_conversion = project2worldFrame(x_pxy, y_pxy, &x_wrld, &y_wrld);
      printf("\n");
      printf("Distorted Pixel Coordinates\n");
      printf("x: %i y: %i \n \n", x_pxy, y_wrld);
      printf("World frame coordinates\n");
      printf("x: %d y: %d \n");
    }
    else if (ball_detected == 0)
    {
      printf("No ball was detected");
    }

    /* ********************* */
    /* Insert your Code here */
    /* ********************* */
  }

  return 0;
}
