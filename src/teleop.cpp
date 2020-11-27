#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_F 0x41
#define KEYCODE_B 0x42
#define KEYCODE_S 0x73

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char **argv)
{
  bool dirty = false;
  bool listen = true;
  char key;

  geometry_msgs::Twist vel_cmd;

  ros::init(argc, argv, "teleop");

  ros::NodeHandle nh;
  ros::Publisher vel_cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

  ros::Rate read_rate = 100;  // (Hz)

  printf("Reading from keyboard\r\n");
  printf("---------------------------\r\n");
  printf("Use arrow keys to move the robot; 's' stops the bot.\r\n");

  while (listen)
  {
    // Get the pressed key
    key = getch();

    switch (key)
    {
      case KEYCODE_F:
        printf("FORWARD\r\n");
        vel_cmd.linear.x = 0.5;
        vel_cmd.angular.z = 0.0;
        dirty = true;
        break;
      case KEYCODE_B:
        printf("BACK\r\n");
        vel_cmd.linear.x = -0.5;
        vel_cmd.angular.z = 0.0;
        dirty = true;
        break;
      case KEYCODE_L:
        printf("LEFT\r\n");
        vel_cmd.linear.x = 0.0;
        vel_cmd.angular.z = 1.8;
        dirty = true;
        break;
      case KEYCODE_R:
        printf("RIGHT\r\n");
        vel_cmd.linear.x = 0.0;
        vel_cmd.angular.z = -1.8;
        dirty = true;
        break;
      case '\x03':
        printf("\r\nStopping the robot and ending the teleop program\r\n");
        listen = false;
      case 's':
        printf("STOP\r\n");
        vel_cmd.linear.x = 0.0;
        vel_cmd.angular.z = 0.0;
        dirty = true;
        break;
      default:
        break;
    }

    if (dirty == true)
    {
      vel_cmd_pub.publish(vel_cmd);
      dirty = false;
    }

    read_rate.sleep();
    ros::spinOnce();
  }

  ros::shutdown();
  return 0;
}
