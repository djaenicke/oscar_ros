#include "ros/ros.h"

#include <nav_msgs/Odometry.h>
#include <oscar_pi/cmd.h>
#include "oscar_ros/footprint.h"
#include <oscar_pi/state.h>
#include <sensor_msgs/Imu.h>
#include <vector>

static void stateMsgUpdateCallBack(const oscar_pi::state::ConstPtr& msg);

// Pubs and Subs
static ros::Subscriber state_sub;
static ros::Publisher odom_raw_pub;
static ros::Publisher footprint_pub;
static ros::Publisher imu_mpu_pub;

static nav_msgs::Odometry odom_raw;
static sensor_msgs::Imu mpu;
static geometry_msgs::PolygonStamped robot_polygon;

static ros::Time current_time, last_time;

// Calibrations
static float wheel_base_m;  // (m)
static float wheel_radius_m;  // (m)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry");
  ros::NodeHandle nh;

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  state_sub = nh.subscribe("robot_state", 100, stateMsgUpdateCallBack);

  odom_raw_pub = nh.advertise<nav_msgs::Odometry>("odom/data_raw", 100);
  footprint_pub = nh.advertise<geometry_msgs::PolygonStamped>("robot_footprint", 100);
  imu_mpu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 100);

  // Configure the robot's footprint for rviz
  int num_points;
  nh.getParam("/odometry/robot_footprint_points", num_points);
  oscar_ros::Footprint robot_footprint = oscar_ros::Footprint(num_points);

  std::vector<float> points_x;
  std::vector<float> points_y;
  nh.getParam("/odometry/robot_footprint_x", points_x);
  nh.getParam("/odometry/robot_footprint_y", points_y);

  for (uint8_t i = 0; i < num_points; i++)
  {
    robot_footprint.AddPoint(points_x[i], points_y[i]);
  }
  robot_polygon = robot_footprint.GetPolyStampedMsg();

  // Set parameters
  if (!nh.getParam("/odometry/wheel_base_m", wheel_base_m))
  {
    ROS_ERROR("wheel_base_m rosparam undefined");
    ros::shutdown();
  }

  if (!nh.getParam("/odometry/wheel_radius_m", wheel_radius_m))
  {
    ROS_ERROR("wheel_radius_m rosparam undefined");
    ros::shutdown();
  }

  mpu.header.frame_id = "base_link";

  odom_raw.header.frame_id = "odom";
  odom_raw.child_frame_id = "base_link";

  odom_raw.twist.covariance[0] = 1e-5;   // x with respect to x
  odom_raw.twist.covariance[7] = 1e-6;   // y with respect to y
  odom_raw.twist.covariance[35] = 1e-2;  // rotation about Z axis with respect to rotation about Z axis

  ros::spin();
  return 0;
}

static void stateMsgUpdateCallBack(const oscar_pi::state::ConstPtr& msg)
{
  // Compute time between state messages
  current_time = ros::Time::now();
  const double dt = (current_time - last_time).toSec();
  last_time = current_time;

  // Compute the odometry
  const double vr = msg->r_wheel_fb * wheel_radius_m;  // Right wheel translational velocity
  const double vl = msg->l_wheel_fb * wheel_radius_m;  // Left wheel translational velocity

  const double th_dot = (vr - vl) / wheel_base_m;  // Robot angular velocity
  const double x_dot = (vr + vl) / 2;  // Robot translational velocity

  odom_raw.header.stamp = current_time;
  odom_raw.twist.twist.linear.x = x_dot;
  odom_raw.twist.twist.linear.y = 0;
  odom_raw.twist.twist.angular.z = th_dot;

  odom_raw_pub.publish(odom_raw);
  odom_raw.header.seq++;

  robot_polygon.header.stamp = current_time;
  footprint_pub.publish(robot_polygon);
  robot_polygon.header.seq++;

  mpu.header.stamp = current_time;
  mpu.angular_velocity.x = msg->mpu_gx;
  mpu.angular_velocity.y = msg->mpu_gy;
  mpu.angular_velocity.z = msg->mpu_gz;
  mpu.linear_acceleration.x = msg->mpu_ax;
  mpu.linear_acceleration.y = msg->mpu_ay;
  mpu.linear_acceleration.z = msg->mpu_az;
  imu_mpu_pub.publish(mpu);
  mpu.header.seq++;
}
