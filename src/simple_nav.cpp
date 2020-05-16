#include "ros/ros.h"
#include <signal.h>
#include "robo_car_if/go_to_point.h"

/* Tuning parameters */
#define Kp  (2.5f)
#define TOLERANCE (0.1)  /* (m) */
#define GTP_SPEED (0.5) /* (m/s) */

#define EXE_RATE (0.050)
#define EXES_PER_SEC (1/EXE_RATE)

static ros::Publisher cmd_pub;
static robo_car_if::cmd cmd_msg;

std::vector<robo_car_if::Waypoint_T> waypoints;

void MySigintHandler(int sig);

int main(int argc, char **argv) {
  
  robo_car_if::GoToPointController gtp_controller(TOLERANCE, Kp, GTP_SPEED);

  ros::init(argc, argv, "simple_nav");

  ros::NodeHandle nh;
  cmd_pub = nh.advertise<robo_car_if::cmd>("robo_car_cmd", 100);
  ros::Subscriber odom_sub = nh.subscribe("odom", 100, &robo_car_if::GoToPointController::UpdatePose, &gtp_controller);
  ros::Rate loop_rate(1/EXE_RATE); // (Hz)

  // Override the default ros sigint handler.
  signal(SIGINT, MySigintHandler);

  // Configure the path as a vector of waypoints
  waypoints.push_back({1,1});
  waypoints.push_back({1,0});
  waypoints.push_back({0,0});
  int num_waypoints = waypoints.size();
  int current_wp = 0;

  // Set initial destination
  gtp_controller.UpdateDestination(&waypoints[current_wp]);

  while (ros::ok()) {
    ros::spinOnce();

    if (gtp_controller.InRoute()) {
      gtp_controller.Execute();
      cmd_msg = gtp_controller.GetCmdMsg();
    } else {
      /* Waypoint reached; stop */
      cmd_msg.l_wheel_sp = 0.0;
      cmd_msg.r_wheel_sp = 0.0;
      cmd_msg.stop = 1;

      /* Do we have another waypoint along the path? */
      if (++current_wp < num_waypoints) {
        gtp_controller.UpdateDestination(&waypoints[current_wp]);
      }
    }

    cmd_pub.publish(cmd_msg);
    loop_rate.sleep();
  }

  return 0;
}

void MySigintHandler(int sig)
{
  uint8_t i;

  cmd_msg.l_wheel_sp = 0.0;
  cmd_msg.r_wheel_sp = 0.0;
  cmd_msg.stop = 1;

  for (i=0; i<3; i++) {
    cmd_pub.publish(cmd_msg);
  }

  ros::shutdown();
}