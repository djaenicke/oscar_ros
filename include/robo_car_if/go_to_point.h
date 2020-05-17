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
  float kp_h_;
  bool in_route_ = false;
  Waypoint_T dest_;
  robo_car_if::cmd cmd_;
  float org_dist_to_pnt_;

 public:
  GoToPointController(float tolerance, float kp_v, float kp_h);
  robo_car_if::cmd Execute(void);
  void UpdateDestination(Waypoint_T* dest);
  bool InRoute(void);
  void UpdatePose(const nav_msgs::Odometry::ConstPtr& msg);
};

} // namespace robo_car_if

#endif  // ROBO_CAR_IF_GO_TO_POINT_H_
