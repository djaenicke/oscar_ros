#ifndef ROBO_CAR_ROS_IF_CALS_H_
#define ROBO_CAR_ROS_IF_CALS_H_

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

#define EMBEDDED_UPDATE_RATE 0.050  // (s)
#define WHEEL_BASE   0.225  // (m)
#define WHEEL_RADIUS 0.0335  // (m)

#define MAX_WHEEL_SPEED 30  // (rad/s)
#define MAX_ABS_YAW_RATE (WHEEL_RADIUS / WHEEL_BASE * (2 * MAX_WHEEL_SPEED))

#endif  // ROBO_CAR_ROS_IF_CALS_H_
