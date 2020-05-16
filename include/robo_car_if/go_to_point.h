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
} Destination_T;

typedef struct {
  float x;
  float y;
  float theta;
} Pose_T;

class GoToPointController {
 private:
  Pose_T pose_;
  float tol_;
  float kp_;
  bool in_route_ = false;
  Destination_T dest_;
  float heading_sp_;
  float robot_v_;
  robo_car_if::cmd cmd_;

 public:
  GoToPointController(float tolerance, float gain, float travel_s);
  void Execute(void);
  void SetTravelSpeed(float robot_v);
  void UpdateDestination(Destination_T* dest);
  bool InRoute(void);
  robo_car_if::cmd GetCmdMsg(void);
  void UpdatePose(const nav_msgs::Odometry::ConstPtr& msg);
  void GetPose(Pose_T * pose);
};

} // namespace robo_car_if

#endif  // ROBO_CAR_IF_GO_TO_POINT_H_
