#include "ros/ros.h"
#include "robo_car_if/go_to_point.h"

/* Tuning parameters */
#define Kp  (4.0f)
#define TOLERANCE (0.01) /* (m) */
#define GTP_SPEED (0.5)  /* (m/s) */

#define EXE_RATE (0.050)
#define EXES_PER_SEC (1/EXE_RATE)

int main(int argc, char **argv) {
  robo_car_if::cmd cmd_msg;
  robo_car_if::GoToPointController gtp_controller(TOLERANCE, GTP_SPEED, Kp);

  ros::init(argc, argv, "simple_nav");

  ros::NodeHandle nh;
  ros::Publisher cmd_pub = nh.advertise<robo_car_if::cmd>("robo_car_cmd", 100);
  ros::Subscriber odom_sub = nh.subscribe("odom", 100, &robo_car_if::GoToPointController::UpdatePose, &gtp_controller);
  ros::Rate loop_rate(1/EXE_RATE); // (Hz)

  // Set initial destination
  robo_car_if::Destination_T dest;
  dest.x = 1;
  dest.y = 0;
  gtp_controller.UpdateDestination(&dest);

  robo_car_if::Pose_T pose;
  uint16_t exe_cntr = 0;

  while (ros::ok()) {
    ros::spinOnce();

    if (gtp_controller.InRoute()) {
      gtp_controller.Execute();
      cmd_msg = gtp_controller.GetCmdMsg();
    } else {
      cmd_msg.l_wheel_sp = 0.0;
      cmd_msg.r_wheel_sp = 0.0;
      cmd_msg.stop = 1;
    }

    if (0 == ++exe_cntr % (uint16_t) EXES_PER_SEC) {
      gtp_controller.GetPose(&pose);
      ROS_INFO("position = (%0.2f, %0.2f) direction = %0.2f", pose.x, pose.y, pose.theta);
      exe_cntr = 0;
    }

    //cmd_pub.publish(cmd_msg);
    loop_rate.sleep();
  }

  return 0;
}