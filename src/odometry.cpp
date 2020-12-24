#include "ros/ros.h"
#include "robo_car_ros_if/footprint.h"
#include "robot_localization/SetPose.h"

#include <oscar_pi/state.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <signal.h>
#include <vector>

static void StateMsgUpdateCallBack(const oscar_pi::state::ConstPtr& msg);

// Pubs and Subs
static ros::Subscriber state_sub;
static ros::Publisher odom_pub;
static ros::Publisher footprint_pub;
static ros::Publisher imu_mpu_pub;

// ServiceClients
static ros::ServiceClient reset_ekf_pose;

static nav_msgs::Odometry odom;
static sensor_msgs::Imu mpu;
static geometry_msgs::PolygonStamped robot_polygon;

static ros::Time current_time, last_time;

// Calibrations
static float wheel_base;  // (m)
static float wheel_radius;  // (m)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_estimation");
  ros::NodeHandle nh;

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  state_sub = nh.subscribe("robot_state", 100, StateMsgUpdateCallBack);

  reset_ekf_pose = nh.serviceClient<robot_localization::SetPose>("/set_pose");

  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
  footprint_pub = nh.advertise<geometry_msgs::PolygonStamped>("robot_footprint", 100);
  imu_mpu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 100);

  // Configure the robot's footprint for rviz
  int num_points;
  nh.getParam("/odometry/robot_footprint_points", num_points);
  robo_car_ros_if::Footprint robot_footprint = robo_car_ros_if::Footprint(num_points);

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
  if (!nh.getParam("/odometry/wheel_base", wheel_base))
  {
    ROS_ERROR("wheel_base rosparam undefined");
    ros::shutdown();
  }

  if (!nh.getParam("/odometry/wheel_radius", wheel_radius))
  {
    ROS_ERROR("wheel_radius rosparam undefined");
    ros::shutdown();
  }

  mpu.header.frame_id = "base_link";

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.twist.covariance[0] = 1e-5;   // x with respect to x
  odom.twist.covariance[7] = 1e-6;   // y with respect to y
  odom.twist.covariance[35] = 1e-2;  // rotation about Z axis with respect to rotation about Z axis

  ros::spin();
  return 0;
}

static void StateMsgUpdateCallBack(const oscar_pi::state::ConstPtr& msg)
{
  // Compute time between state messages
  current_time = ros::Time::now();
  const double dt = (current_time - last_time).toSec();
  last_time = current_time;

  // Compute the odometry
  const double vr = msg->r_wheel_fb * wheel_radius;  // Right wheel translational velocity
  const double vl = msg->l_wheel_fb * wheel_radius;  // Left wheel translational velocity

  const double th_dot = (vr - vl) / wheel_base;  // Robot angular velocity
  const double x_dot = (vr + vl) / 2;  // Robot translational velocity

  odom.header.stamp = current_time;
  odom.twist.twist.linear.x = x_dot;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = th_dot;

  odom_pub.publish(odom);
  odom.header.seq++;

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
