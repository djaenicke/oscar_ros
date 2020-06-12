/*
 * go_to_point.h
 *
 *  Created on: Jul 28, 2019
 *      Author: Devin
 */

#ifndef ROBO_CAR_IF_GO_TO_POINT_H_
#define ROBO_CAR_IF_GO_TO_POINT_H_

#include "ros/ros.h"
#include "robo_car_if/cmd.h"
#include "robo_car_if/pid.h"
#include <nav_msgs/Odometry.h>

namespace robo_car_if {

typedef struct {
  float x;
  float y;
} Waypoint_T;

typedef struct {
  float x;
  float y;
  float theta;
} Pose_T;

class GoToPointController {
 private:
  Pose_T pose_;
  float tol_;
  float kp_v_;
  float ff_v_;
  bool in_route_ = false;
  bool aligned_ = false;
  uint8_t delay_cnt_;
  Waypoint_T dest_;
  robo_car_if::cmd cmd_;
  float org_dist_to_pnt_;
  pid::Params_T heading_pid_params;
  pid::PID heading_pid;

 public:
  GoToPointController(float d_tol, float kp_v, float ff_v,
                      pid::Params_T* heading_ctrl_params);
  robo_car_if::cmd Execute(void);
  void UpdateDestination(Waypoint_T* dest);
  bool InRoute(void);
  void UpdatePose(const nav_msgs::Odometry::ConstPtr& msg);
};

} // namespace robo_car_if

#endif  // ROBO_CAR_IF_GO_TO_POINT_H_
