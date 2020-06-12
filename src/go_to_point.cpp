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

GoToPointController::GoToPointController(float d_tol, float kp_v,
                                         float ff_v, pid::Params_T* heading_ctrl_params) :
  heading_pid(heading_ctrl_params) {
  memcpy(&heading_pid_params, heading_ctrl_params, sizeof(pid::Params_T));
  tol_  = d_tol;
  kp_v_ = kp_v;
  ff_v_ = ff_v;
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

  /* Compute the desired heading */
  sp = atan2f(dest_.y - pose_.y, dest_.x - pose_.x);

  /* Compute the distance from the destination */
  d = sqrt(pow(dest_.x - pose_.x, 2) + pow(dest_.y - pose_.y, 2));

  /* Heading difference and error */
  theta_d = sp - pose_.theta; 
  theta_e = atan2f(sinf(theta_d), cosf(theta_d));

  /* Rotate in place to first align the heading angle */
  if (!aligned_) {
    omega = heading_pid.Step(theta_e, heading_pid_params.max, heading_pid_params.min);
    robot_v = 0;

    if (fabs(omega) < 0.4f) {
      delay_cnt_++;
    } else {
      delay_cnt_ = 0;
    }

    if (delay_cnt_ >= 5 && theta_e < 0.1) {
      aligned_ = true;
    }
  }

  if (aligned_) {
    /* Run basic proportional control */
    omega = heading_pid_params.kp * theta_e;
  
    /* Compute the linear velocity */
    /* The distance to the point is normalized to ensure the max possible
      speed is simply kp_v_ + feedforward velocity */ 
    robot_v = (kp_v_ * d / org_dist_to_pnt_) + ff_v_;
  }

  if (d > tol_) {
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
  in_route_ = true;
  aligned_ = false;
  delay_cnt_ = 0;
  heading_pid.Reset();
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