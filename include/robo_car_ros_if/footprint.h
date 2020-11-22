#ifndef ROBO_CAR_ROS_IF_FOOTPRINT_H
#define ROBO_CAR_ROS_IF_FOOTPRINT_H

#include "ros/ros.h"
#include <ros/console.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>

namespace robo_car_ros_if
{
typedef enum
{
  RR = 0,
  RL,
  FL,
  FR,
  NUM_POINTS
} Point_Loc_T;  // NOLINT(whitespace/braces)

class Footprint
{
 private:
  geometry_msgs::PolygonStamped polygon_msg_;
  geometry_msgs::Point32 point_;

 public:
  Footprint()
  {
    polygon_msg_.header.frame_id = "base_link";
    polygon_msg_.polygon.points.reserve(NUM_POINTS);
    point_.x = 0.0f;
    point_.y = 0.0f;
    point_.z = 0.0f;

    for (uint8_t i = 0; i < NUM_POINTS; i++)
    {
      polygon_msg_.polygon.points.push_back(point_);
    }
  }

  void SetPoint(Point_Loc_T loc, float x, float y)
  {
    geometry_msgs::Point32 point;

    switch (loc)
    {
      case RR:
      case RL:
      case FL:
      case FR:
        polygon_msg_.polygon.points[loc].x = x;
        polygon_msg_.polygon.points[loc].y = y;
        break;
      default:
        break;
    }
  }

  geometry_msgs::PolygonStamped GetPolyStampedMsg(void)
  {
    return polygon_msg_;
  }
};

}  // namespace robo_car_ros_if

#endif  // ROBO_CAR_ROS_IF_FOOTPRINT_H
