#include "ros/ros.h"
#include <signal.h>
#include <vector>
#include "robo_car_ros_if/cals.h"
#include "robo_car_ros_if/go_to_point.h"
#include <visualization_msgs/Marker.h>

/* Tuning parameters */
#define D_TOL     (0.05f)   /* Distance to goal tolerance (m) */
#define H_TOL     (0.10f)   /* Angle tolerance when rotating in place (rad) */
#define KP_V      (0.4f)    /* linear velocity P gain */
#define FF_V      (0.25f)   /* linear velocity feedforward term */
#define KP_H      (4.5f)    /* heading angle P gain   */
#define KI_H      (0.0f)    /* heading angle I gain   */
#define KD_H      (0.0f)    /* heading angle D gain   */

#define EXE_RATE (0.050)
#define EXES_PER_SEC (1/EXE_RATE)

static ros::Publisher cmd_pub;
static robo_car_ros_if::cmd cmd_msg;

static void SigintHandler(int sig);
static void RobotForceStop(void);

int main(int argc, char **argv)
{
  robo_car_ros_if::GTP_Cfg_T gtp_cfg;

  gtp_cfg.d_tol = D_TOL;
  gtp_cfg.h_tol = H_TOL;
  gtp_cfg.kp_v  = KP_V;
  gtp_cfg.ff_v  = FF_V;
  gtp_cfg.kp_h  = KP_H;
  gtp_cfg.max_h_dot = MAX_ABS_YAW_RATE / 4;  // Limit max rotational speed
  gtp_cfg.min_h_dot = 0.4;  // (rad/s)

  robo_car_ros_if::GoToPointController gtp_controller(&gtp_cfg);
  std::vector<robo_car_ros_if::Waypoint_T> waypoints;
  int current_wp = 0;
  bool dest_reached = false;
  visualization_msgs::Marker points;
  geometry_msgs::Point p;

  ros::init(argc, argv, "simple_nav");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("odometry/filtered", 100,
                                          &robo_car_ros_if::GoToPointController::UpdatePose, &gtp_controller);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Rate loop_rate(1 / EXE_RATE);  // (Hz)

  cmd_pub = nh.advertise<robo_car_ros_if::cmd>("robo_car_cmd", 100);

  // Override the default ros sigint handler.
  signal(SIGINT, SigintHandler);

  // Configure the path as a vector of waypoints
  waypoints.push_back({1.0, 0.0});
  waypoints.push_back({1.0, -1.0});
  waypoints.push_back({0.0, -1.0});
  waypoints.push_back({0.0, 0.0});

  points.header.frame_id = "odom";
  points.header.stamp = ros::Time::now();
  points.ns = "simple_nav";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.05;
  points.scale.y = 0.05;
  points.color.g = 1.0f;  // Points are green
  points.color.a = 1.0;

  for (auto it = waypoints.begin(); it < waypoints.end(); it++)
  {
    p.x = it->x;
    p.y = it->y;
    p.z = 0.0f;
    points.points.push_back(p);
  }

  // Set initial destination
  gtp_controller.UpdateDestination(&waypoints[current_wp]);

  while (ros::ok() && !dest_reached)
  {
    ros::spinOnce();

    if (gtp_controller.InRoute())
    {
      cmd_msg = gtp_controller.Execute();
    }
    else
    {
      /* Waypoint reached; stop */
      RobotForceStop();

      /* Do we have another waypoint along the path? */
      if (++current_wp < waypoints.size())
      {
        gtp_controller.UpdateDestination(&waypoints[current_wp]);
        ros::Duration(0.1).sleep();
      }
      else
      {
        dest_reached = true;
      }
    }

    points.header.stamp = ros::Time::now();
    marker_pub.publish(points);
    cmd_pub.publish(cmd_msg);
    loop_rate.sleep();
  }

  return 0;
}

static void SigintHandler(int sig)
{
  RobotForceStop();
  ros::shutdown();
}

static void RobotForceStop(void)
{
  cmd_msg.l_wheel_sp = 0.0;
  cmd_msg.r_wheel_sp = 0.0;
  cmd_msg.stop = 1;

  /* Send multiple times just to be sure */
  for (uint8_t i = 0; i < 5; i++)
  {
    cmd_pub.publish(cmd_msg);
    ros::Duration(EXE_RATE).sleep();
  }
}
