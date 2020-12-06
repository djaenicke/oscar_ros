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

typedef enum {
  IDLE,
  ALIGN_HEADING,
  TRANSLATE
} GTP_State_T;

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
  float max_heading_err;  // Maximum heading error allowed before starting to translate to goal
  float max_dist_err;  // Maximum linear distance allowed before considering the destination met
  float kp_h_align;  // Heading angle proprortional gain during the align state
  float kp_h_translate;  // Heading angle proprortional gain during the translate state
  float kp_d;  // Linear distance proprortional gain during the translate state
  float ff_v;  // Feedforward velocity during the translate state
} GTP_Cfg_T;  // NOLINT(whitespace/braces)

class GoToPointController
{
 private:
  Pose_T pose_;
  GTP_State_T state_;
  GTP_Cfg_T cfg_;
  Waypoint_T dest_;
  float heading_sp_;
  geometry_msgs::Twist cmd_;
  float org_dist_to_pnt_;

  void AlignHeading(const float heading_error);
  void Translate(const float heading_error);

 public:
  explicit GoToPointController(const GTP_Cfg_T *const cfg);
  geometry_msgs::Twist Execute(void);
  void UpdateDestination(const Waypoint_T *const dest);
  void UpdatePose(const nav_msgs::Odometry::ConstPtr& msg);
};

}  // namespace robo_car_ros_if

#endif  // ROBO_CAR_ROS_IF_GO_TO_POINT_H_
