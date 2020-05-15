#include "ros/ros.h"
#include "robo_car_if/state.h"
#include "robo_car_if/cmd.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_F 0x41
#define KEYCODE_B 0x42
#define KEYCODE_S 0x73

int main(int argc, char **argv) {
  char c;
  bool dirty = false;
  int kfd = 0;
  struct termios cooked, raw;

  robo_car_if::cmd cmd_msg;

  ros::init(argc, argv, "robo_car_if");

  ros::NodeHandle nh;
  ros::Publisher cmd_pub = nh.advertise<robo_car_if::cmd>("robo_car_cmd", 100);

  ros::Rate loop_rate(1/0.025); // Loop rate in Hz

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");

  while (ros::ok()) {
    // get the next event from the keyboard
    if (read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }
    switch (c) {
      case KEYCODE_F:
        puts("FORWARD");
        cmd_msg.r_wheel_sp = 15.0;
        cmd_msg.l_wheel_sp = 15.0;
        cmd_msg.stop = 0;
        dirty = true;
        break;
      case KEYCODE_B:
        puts("BACK");
        cmd_msg.r_wheel_sp = -15.0;
        cmd_msg.l_wheel_sp = -15.0;
        cmd_msg.stop = 0;
        dirty = true;
        break;
      case KEYCODE_L:
        puts("LEFT");
        cmd_msg.r_wheel_sp = -6.0;
        cmd_msg.l_wheel_sp =  6.0;
        cmd_msg.stop = 0;
        dirty = true;
        break;
      case KEYCODE_R:
        puts("RIGHT");
        cmd_msg.r_wheel_sp =  6.0;
        cmd_msg.l_wheel_sp = -6.0;
        cmd_msg.stop = 0;
        dirty = true;
        break;
      case KEYCODE_S:
        puts("STOP");
        cmd_msg.r_wheel_sp = 0.0;
        cmd_msg.l_wheel_sp = 0.0;
        cmd_msg.stop = 1;
        dirty = true;
        break;
    }

    if (dirty == true) {
      cmd_pub.publish(cmd_msg);
      dirty = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
 