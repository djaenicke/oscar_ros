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

GoToPointController::GoToPointController(float tolerance, float gain, float travel_s) {
  tol_ = tolerance;
  kp_ = gain;
  robot_v_ = travel_s;
  in_route_ = false;
}

void GoToPointController::Execute(void) {
  float vr, vl;           /* Desired linear wheel velocities */
  float theta_d, theta_e; /* Heading difference and heading error */
  float omega;
  float d; /* distance to destination */

  /* Compute the distance from the destination */
  d = sqrt(pow(dest_.x - pose_.x, 2) + pow(dest_.y - pose_.y, 2));

  if (d > tol_) {
    theta_d = heading_sp_ - pose_.theta;           /* Heading difference */
    theta_e = atan2f(sinf(theta_d), cosf(theta_d)); /* Heading error      */

    /* Proportional control */
    omega = kp_ * theta_e;

    /* Determine the required linear velocities */
    vr = robot_v_ + (omega * WHEEL_BASE / 2);
    vl = robot_v_ - (omega * WHEEL_BASE / 2);

    cmd_.r_wheel_sp = vr / WHEEL_RADIUS;
    cmd_.r_wheel_sp = vl / WHEEL_RADIUS;

  } else {
    in_route_ = false;
  }
}

void GoToPointController::SetTravelSpeed(float robot_v) {
  /* Linear velocity in (m/s) */
  robot_v_ = robot_v;
}

void GoToPointController::UpdateDestination(Destination_T * dest) {
  dest_.x = dest->x;
  dest_.y = dest->y;

  /* Compute the desired heading */
  heading_sp_ = atan2f(dest_.x, dest_.y);
  in_route_ = true;
}

bool GoToPointController::InRoute(void){
  return (in_route_);
}

robo_car_if::cmd GoToPointController::GetCmdMsg(void) { 
  return cmd_;
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