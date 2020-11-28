#include "ros/ros.h"
#include "robo_car_ros_if/go_to_point.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>


/* Tuning parameters */
#define D_TOL     (0.05f)   /* Distance to goal tolerance (m) */
#define H_TOL     (0.2f)    /* Angle tolerance when rotating in place (rad) */
#define KP_V      (0.4f)    /* linear velocity P gain */
#define FF_V      (0.5f)    /* linear velocity feedforward term */
#define KP_H      (12.5f)   /* heading angle P gain   */

static void NavGoalUpdateCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);

// Pubs and Subs
static ros::Subscriber nav_goal_sub;
static ros::Publisher vel_cmd_pub;

static robo_car_ros_if::Waypoint_T dest;
static bool new_dest = false;

static visualization_msgs::Marker dest_marker;

int main(int argc, char **argv)
{
  robo_car_ros_if::GTP_Cfg_T gtp_cfg;

  gtp_cfg.d_tol = D_TOL;
  gtp_cfg.h_tol = H_TOL;
  gtp_cfg.kp_v  = KP_V;
  gtp_cfg.ff_v  = FF_V;
  gtp_cfg.kp_h  = KP_H;
  gtp_cfg.max_h_dot = 2.0;  // Limit max rotational speed
  gtp_cfg.min_h_dot = 0.8;  // (rad/s)

  robo_car_ros_if::GoToPointController gtp_controller(&gtp_cfg);
  ros::init(argc, argv, "rviz_nav");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);  // (Hz)
  geometry_msgs::Twist vel_cmd;

  ros::Subscriber odom_sub = nh.subscribe("odometry/filtered", 100,
                                          &robo_car_ros_if::GoToPointController::UpdatePose, &gtp_controller);
  nav_goal_sub = nh.subscribe("/move_base_simple/goal", 100, NavGoalUpdateCallBack);
  vel_cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  dest_marker.ns = "simple_nav";
  dest_marker.action = visualization_msgs::Marker::ADD;
  dest_marker.pose.orientation.w = 1.0;
  dest_marker.id = 0;
  dest_marker.type = visualization_msgs::Marker::SPHERE;
  dest_marker.scale.x = 0.1;
  dest_marker.scale.y = 0.1;
  dest_marker.scale.z = 0.05;
  dest_marker.color.g = 1.0f;  // Points are green
  dest_marker.color.a = 1.0;

  while (ros::ok())
  {
    ros::spinOnce();

    if (new_dest)
    {
      ROS_INFO("New goal");
      gtp_controller.UpdateDestination(&dest);
      new_dest = false;
    }

    if (gtp_controller.InRoute())
    {
      vel_cmd = gtp_controller.Execute();
    }
    else
    {
      vel_cmd.linear.x = 0.0;
      vel_cmd.angular.z = 0.0;
    }

    dest_marker.header.stamp = ros::Time::now();
    vis_pub.publish(dest_marker);
    vel_cmd_pub.publish(vel_cmd);
    loop_rate.sleep();
  }

  return 0;
}

static void NavGoalUpdateCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // frame_id is set based on the fixed_frame in RViz
  dest_marker.header.frame_id = msg->header.frame_id;
  dest_marker.pose = msg->pose;

  new_dest = true;
  dest.x = msg->pose.position.x;
  dest.y = msg->pose.position.y;
}
