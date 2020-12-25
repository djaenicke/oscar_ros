#include "ros/ros.h"

#include <oscar_pi/cmd.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <sensor_msgs/Range.h>

static void VelocityCmdUpdateCallBack(const geometry_msgs::Twist::ConstPtr& msg);
static void RangeSensorUpdateCallBack(const sensor_msgs::Range::ConstPtr& msg);

// Pubs and Subs
static ros::Subscriber vel_cmd_sub;
static ros::Publisher robot_cmd_pub;
static ros::Subscriber range_sensor_sub;

// Calibrations
static const float stop_threshold = 0.001;
static float wheel_base;  // (m)
static float wheel_radius;  // (m)

// Constraints
static float max_wheel_speed;  // (rad/s)

static oscar_pi::cmd cmd;
static bool obj_detected = true;
static bool moving_fwd = false;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_cmd");
  ros::NodeHandle nh;

  if (!nh.getParam("/velocity_cmd/wheel_base", wheel_base))
  {
    ROS_ERROR("wheel_base rosparam undefined");
    ros::shutdown();
  }

  if (!nh.getParam("/velocity_cmd/wheel_radius", wheel_radius))
  {
    ROS_ERROR("wheel_radius rosparam undefined");
    ros::shutdown();
  }

  if (!nh.getParam("/velocity_cmd/max_wheel_speed", max_wheel_speed))
  {
    ROS_ERROR("max_wheel_speed rosparam undefined");
    ros::shutdown();
  }

  if (!nh.getParam("/velocity_cmd/r_kp", cmd.r_kp))
  {
    ROS_ERROR("r_kp rosparam undefined");
    ros::shutdown();
  }

  if (!nh.getParam("/velocity_cmd/r_ki", cmd.r_ki))
  {
    ROS_ERROR("r_ki rosparam undefined");
    ros::shutdown();
  }

  if (!nh.getParam("/velocity_cmd/r_kd", cmd.r_kd))
  {
    ROS_ERROR("r_kd rosparam undefined");
    ros::shutdown();
  }

  if (!nh.getParam("/velocity_cmd/l_kp", cmd.l_kp))
  {
    ROS_ERROR("l_kp rosparam undefined");
    ros::shutdown();
  }

  if (!nh.getParam("/velocity_cmd/l_ki", cmd.l_ki))
  {
    ROS_ERROR("l_ki rosparam undefined");
    ros::shutdown();
  }

  if (!nh.getParam("/velocity_cmd/l_kd", cmd.l_kd))
  {
    ROS_ERROR("l_kd rosparam undefined");
    ros::shutdown();
  }

  if (!nh.getParam("/velocity_cmd/wheel_speed_filt_alpha", cmd.wheel_speed_filt_alpha))
  {
    ROS_ERROR("wheel_speed_filt_alpha rosparam undefined");
    ros::shutdown();
  }

  vel_cmd_sub = nh.subscribe("/cmd_vel", 100, VelocityCmdUpdateCallBack);
  robot_cmd_pub = nh.advertise<oscar_pi::cmd>("/robot_cmd", 100);
  range_sensor_sub = nh.subscribe("/fwd_uss", 100, RangeSensorUpdateCallBack);

  ros::spin();

  return 0;
}

static void VelocityCmdUpdateCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd.r_wheel_sp = (msg->linear.x + ((msg->angular.z * wheel_base) / 2)) / wheel_radius;
  cmd.l_wheel_sp = (msg->linear.x - ((msg->angular.z * wheel_base) / 2)) / wheel_radius;

  if (msg->linear.x > 0)
  {
    moving_fwd = true;
  }
  else
  {
    moving_fwd = false;
  }

  // Saturate the wheel rotational velocities
  if (fabs(cmd.l_wheel_sp) > max_wheel_speed)
  {
    if (cmd.l_wheel_sp > 0)
    {
      cmd.l_wheel_sp = max_wheel_speed;
    }
    else
    {
      cmd.l_wheel_sp = -max_wheel_speed;
    }
  }

  if (fabs(cmd.r_wheel_sp) > max_wheel_speed)
  {
    if (cmd.r_wheel_sp > 0)
    {
      cmd.r_wheel_sp = max_wheel_speed;
    }
    else
    {
      cmd.r_wheel_sp = -max_wheel_speed;
    }
  }

  // Are we stopping?
  if ((fabs(cmd.r_wheel_sp) > stop_threshold) || (fabs(cmd.l_wheel_sp) > stop_threshold))
  {
    cmd.stop = 0;
  }
  else
  {
    cmd.stop = 1;
  }

  if (obj_detected && moving_fwd)
  {
    cmd.stop = 1;
  }

  robot_cmd_pub.publish(cmd);
}

static void RangeSensorUpdateCallBack(const sensor_msgs::Range::ConstPtr& msg)
{
  if ((msg->range > msg->min_range) && (msg->range < msg->max_range))
  {
    // Range is valid
    if (moving_fwd && (msg->range < 0.125))
    {
      // Object in path
      obj_detected = true;
      cmd.stop = 1;
      robot_cmd_pub.publish(cmd);
    }
    else
    {
      obj_detected = false;
    }
  }
}
