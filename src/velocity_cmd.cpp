#include "ros/ros.h"

#include <oscar_pi/cmd.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>

static void VelocityCmdUpdateCallBack(const geometry_msgs::Twist::ConstPtr& msg);

// Pubs and Subs
static ros::Subscriber vel_cmd_sub;
static ros::Publisher robot_cmd_pub;

// Calibrations
static const float stop_threshold = 0.001;
static float wheel_base;  // (m)
static float wheel_radius;  // (m)

// Constraints
static float max_wheel_speed;  // (rad/s)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_cmd");
  ros::NodeHandle nh;

  if (!nh.getParam("wheel_base", wheel_base))
  {
    ROS_ERROR("wheel_base rosparam undefined");
    ros::shutdown();
  }

  if (!nh.getParam("wheel_radius", wheel_radius))
  {
    ROS_ERROR("wheel_radius rosparam undefined");
    ros::shutdown();
  }

  if (!nh.getParam("max_wheel_speed", max_wheel_speed))
  {
    ROS_ERROR("max_wheel_speed rosparam undefined");
    ros::shutdown();
  }

  vel_cmd_sub = nh.subscribe("/cmd_vel", 100, VelocityCmdUpdateCallBack);
  robot_cmd_pub = nh.advertise<oscar_pi::cmd>("robot_cmd", 100);

  ros::spin();
}

static void VelocityCmdUpdateCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{
  oscar_pi::cmd cmd;

  cmd.r_wheel_sp = (msg->linear.x + ((msg->angular.z * wheel_base) / 2)) / wheel_radius;
  cmd.l_wheel_sp = (msg->linear.x - ((msg->angular.z * wheel_base) / 2)) / wheel_radius;

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

  robot_cmd_pub.publish(cmd);
}
