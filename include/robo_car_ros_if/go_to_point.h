/*
 * go_to_point.h
 *
 *  Created on: Jul 28, 2019
 *      Author: Devin
 */

#ifndef ROBO_CAR_ROS_IF_GO_TO_POINT_H_
#define ROBO_CAR_ROS_IF_GO_TO_POINT_H_

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace robo_car_ros_if
{

typedef struct
{
  float x;
  float y;
} Waypoint_T;  // NOLINT(whitespace/braces)

typedef struct
{
  float x;
  float y;
  float theta;
} Pose_T;  // NOLINT(whitespace/braces)

typedef struct
{
  float d_tol;
  float h_tol;
  float kp_v;
  float ff_v;
  float kp_h;
  float max_h_dot;
  float min_h_dot;
} GTP_Cfg_T;  // NOLINT(whitespace/braces)

class GoToPointController
{
 private:
  Pose_T pose_;
  float d_tol_;
  float h_tol_;
  float kp_v_;
  float ff_v_;
  float kp_h_;
  float max_h_dot_;
  float min_h_dot_;
  bool in_route_ = false;
  bool aligned_ = false;
  uint8_t delay_cnt_;
  Waypoint_T dest_;
  geometry_msgs::Twist cmd_;
  float org_dist_to_pnt_;

 public:
  explicit GoToPointController(GTP_Cfg_T* cfg);
  geometry_msgs::Twist Execute(void);
  void UpdateDestination(Waypoint_T* dest);
  bool InRoute(void);
  void UpdatePose(const nav_msgs::Odometry::ConstPtr& msg);
};

}  // namespace robo_car_ros_if

#endif  // ROBO_CAR_ROS_IF_GO_TO_POINT_H_
