/*
 * go_to_point.cpp
 *
 *  Created on: Jul 28, 2019
 *      Author: Devin
 */

#include "robo_car_if/go_to_point.h"
#include "robo_car_if/cals.h"
#include <tf/LinearMath/Matrix3x3.h>

/* Controller Design */
/* https://pdfs.semanticscholar.org/edde/fa921e26efbbfd6c65ad1e13af0bbbc1b946.pdf */
namespace robo_car_if {

GoToPointController::GoToPointController(GTP_Cfg_T* cfg) {
  d_tol_ = cfg->d_tol;
  h_tol_ = cfg->h_tol;
  kp_v_  = cfg->kp_v;
  ff_v_  = cfg->ff_v;
  kp_h_  = cfg->kp_h;
  min_h_dot_ = cfg->min_h_dot;
  max_h_dot_ = cfg->max_h_dot;
  in_route_ = false;
  pose_.x = 0;
  pose_.y = 0;
  pose_.theta = 0;
}

robo_car_if::cmd GoToPointController::Execute(void) {
  float vr, vl;           /* Desired linear wheel velocities */
  float robot_v;          /* Robot linear velocity */
  float theta_d, theta_e; /* Heading difference and heading error */
  float omega;
  float sp;
  float d;
  int8_t sign;

  /* Compute the desired heading */
  sp = atan2f(dest_.y - pose_.y, dest_.x - pose_.x);

  /* Compute the distance from the destination */
  d = sqrt(pow(dest_.x - pose_.x, 2) + pow(dest_.y - pose_.y, 2));

  /* Heading difference and error */
  theta_d = sp - pose_.theta; 
  theta_e = atan2f(sinf(theta_d), cosf(theta_d));
  omega = kp_h_ * theta_e;

  /* Determine the sign of omega */
  sign = signbit(omega) ? -1 : 1;

  /* Align heading angle before translating */
  if (!aligned_) {
    robot_v = 0;

    /* Saturate the yaw rate */
    if (fabs(omega) > max_h_dot_) {
      omega = sign * max_h_dot_;
    }

    if (fabs(omega) < min_h_dot_) {
      delay_cnt_++;
    } else {
      delay_cnt_ = 0;
    }

    if (delay_cnt_ >= 5 && theta_e < h_tol_) {
      aligned_ = true;
    }
  }

  if (aligned_) {
    /* Compute the linear velocity */
    /* The distance to the point is normalized to ensure the max possible
      speed is simply kp_v_ + feedforward velocity */ 
    robot_v = (kp_v_ * d / org_dist_to_pnt_) + ff_v_;
  }

  if (d > d_tol_) {
    /* Determine the required linear velocities */
    vr = robot_v + (omega * (WHEEL_BASE / 2));
    vl = robot_v - (omega * (WHEEL_BASE / 2));

    cmd_.r_wheel_sp = vr / WHEEL_RADIUS;
    cmd_.l_wheel_sp = vl / WHEEL_RADIUS;
  } else {
    in_route_ = false;
  }

  return cmd_;
}

void GoToPointController::UpdateDestination(Waypoint_T* dest) {
  dest_.x = dest->x;
  dest_.y = dest->y;
  org_dist_to_pnt_ = sqrt(pow(dest_.x - pose_.x, 2) + pow(dest_.y - pose_.y, 2));
  in_route_  = true;
  aligned_   = false;
  delay_cnt_ = 0;
}

bool GoToPointController::InRoute(void){
  return (in_route_);
}

void GoToPointController::UpdatePose(const nav_msgs::Odometry::ConstPtr& msg) {
  pose_.x = msg->pose.pose.position.x;
  pose_.y = msg->pose.pose.position.y;

  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  pose_.theta = yaw;
}

} // namespace robo_car_if