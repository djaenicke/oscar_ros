/*
 * go_to_point.cpp
 *
 *  Created on: Jul 28, 2019
 *      Author: Devin
 */

#include "robo_car_ros_if/go_to_point.h"

#include <ros/console.h>
#include <tf/LinearMath/Matrix3x3.h>

/* Controller Design
   https://pdfs.semanticscholar.org/edde/fa921e26efbbfd6c65ad1e13af0bbbc1b946.pdf */
namespace robo_car_ros_if
{

GoToPointController::GoToPointController(const GTP_Cfg_T *const cfg)
{
  pose_.x = 0;
  pose_.y = 0;
  pose_.theta = 0;

  state_ = IDLE;

  if (NULL != cfg)
  {
    (void)memcpy(&cfg_, cfg, sizeof(GTP_Cfg_T));
  }
}

geometry_msgs::Twist GoToPointController::Execute(void)
{
  if (state_ != IDLE)
  {
    float heading_diff = heading_sp_ - pose_.theta;
    float heading_error = atan2f(sinf(heading_diff), cosf(heading_diff));

    if (ALIGN_HEADING == state_)
    {
      AlignHeading(heading_error);
    }
    else
    {
      Translate(heading_error);
    }
  }
  else
  {
    cmd_.linear.x = 0;
    cmd_.angular.z = 0;
  }

  return cmd_;
}

void GoToPointController::AlignHeading(const float heading_error)
{
  cmd_.linear.x = 0;

  if (fabs(heading_error) > cfg_.max_heading_err)
  {
    cmd_.linear.x = 0;
    cmd_.angular.z = cfg_.kp_h_align * heading_error;
  }
  else
  {
    state_ = TRANSLATE;
    cmd_.linear.x = 0;
    cmd_.angular.z = 0;
  }
}

void GoToPointController::Translate(const float heading_error)
{
  float d = sqrt(pow(dest_.x - pose_.x, 2) + pow(dest_.y - pose_.y, 2));

  if (d > cfg_.max_dist_err)
  {
    /* Compute the linear velocity */
    /* The distance to the point is normalized to ensure the max possible
      speed is simply kp_v_ + feedforward velocity */ 
    cmd_.linear.x = (cfg_.kp_d * (d / org_dist_to_pnt_)) + cfg_.ff_v;
    cmd_.angular.z = cfg_.kp_h_translate * heading_error;
  }
  else
  {
    state_ = IDLE;
    cmd_.linear.x = 0;
    cmd_.angular.z = 0;
  }
}

void GoToPointController::UpdateDestination(const Waypoint_T *const dest)
{
  dest_.x = dest->x;
  dest_.y = dest->y;
  org_dist_to_pnt_ = sqrt(pow(dest_.x - pose_.x, 2) + pow(dest_.y - pose_.y, 2));
  heading_sp_ = atan2f(dest_.y - pose_.y, dest_.x - pose_.x);
  state_  = ALIGN_HEADING;
}

void GoToPointController::UpdatePose(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose_.x = msg->pose.pose.position.x;
  pose_.y = msg->pose.pose.position.y;

  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  pose_.theta = yaw;
}

}  // namespace robo_car_ros_if
