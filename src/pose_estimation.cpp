#include "ros/ros.h"
#include "robo_car_if/state.h"
#include "robo_car_if/footprint.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#define EMBEDDED_UPDATE_RATE 0.050 // (s)
#define WHEEL_BASE   0.128         // (m)
#define WHEEL_RADIUS 0.035         // (m)

void StateMsgUpdateCallBack(const robo_car_if::state::ConstPtr& msg);
void ComputeOdometry(float r_w_speed, float l_w_speed, double dt);

static robo_car_if::Footprint robot_footprint;

// Pubs and Subs
static ros::Subscriber state_sub;
static ros::Publisher odom_pub;
static ros::Publisher footprint_pub;
static ros::Publisher imu_mpu_pub;
static ros::Publisher imu_fxos_pub;
static tf::TransformBroadcaster *odom_broadcaster_ptr;

static ros::Time current_time, last_time;
static double x  = 0;
static double y  = 0;
static double th = 0;

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_estimation");
  ros::NodeHandle nh;

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  state_sub = nh.subscribe("robo_car_state", 100, StateMsgUpdateCallBack);
  
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
  footprint_pub = nh.advertise<geometry_msgs::PolygonStamped>("robot_footprint", 100);
  imu_mpu_pub = nh.advertise<sensor_msgs::Imu>("imu_mpu", 100);
  imu_fxos_pub = nh.advertise<sensor_msgs::Imu>("imu_fxos", 100);

  odom_broadcaster_ptr = new tf::TransformBroadcaster();

  // Configure the robot's footprint as a rectangle for rviz
  robot_footprint.SetPoint(robo_car_if::RR, -0.07, -0.08);
  robot_footprint.SetPoint(robo_car_if::RL, -0.07,  0.08);
  robot_footprint.SetPoint(robo_car_if::FL,  0.15,  0.08);
  robot_footprint.SetPoint(robo_car_if::FR,  0.15, -0.08);

  // Process incoming state messages at 2 times the update rate
  ros::Rate loop_rate(1/(EMBEDDED_UPDATE_RATE/2)); // (Hz)

  while (ros::ok()) {
    ros::spinOnce();
    footprint_pub.publish(robot_footprint.GetPolyStampedMsg());
    loop_rate.sleep();
  }
}

void StateMsgUpdateCallBack(const robo_car_if::state::ConstPtr& msg) {
  sensor_msgs::Imu mpu;
  sensor_msgs::Imu fxos;
  double dt;

  // Compute time between state messages
  current_time = ros::Time::now();
  dt = (current_time - last_time).toSec();
  last_time = current_time;

  ComputeOdometry(msg->l_wheel_fb, msg->r_wheel_fb, dt);

  mpu.header.frame_id = "base_link";
  mpu.header.stamp = current_time;
  mpu.angular_velocity.x = msg->mpu_gx;
  mpu.angular_velocity.y = msg->mpu_gy;
  mpu.angular_velocity.z = msg->mpu_gz;
  mpu.linear_acceleration.x = msg->mpu_ax;
  mpu.linear_acceleration.y = msg->mpu_ay;
  mpu.linear_acceleration.z = msg->mpu_az;
  imu_mpu_pub.publish(mpu);

  fxos.header.frame_id = "base_link";
  fxos.header.stamp = current_time;
  fxos.linear_acceleration.x = msg->fxos_ax;
  fxos.linear_acceleration.y = msg->fxos_ay;
  fxos.linear_acceleration.z = msg->fxos_az;
  imu_fxos_pub.publish(fxos);
}

void ComputeOdometry(float l_w_speed, float r_w_speed, double dt) {
  double vr = r_w_speed * WHEEL_RADIUS; // Right wheel translational velocity
  double vl = l_w_speed * WHEEL_RADIUS; // Right wheel translational velocity
  double yaw_rate = (vr - vl) / WHEEL_BASE; // Robot angular velocity
  double v = (vr + vl) / 2; // Robot translational velocity
  double delta_x = (v * cos(th)) * dt;
  double delta_y = (v * sin(th)) * dt;
  double delta_th = yaw_rate * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster_ptr->sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = v;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = yaw_rate;

  //publish the message
  odom_pub.publish(odom);
}