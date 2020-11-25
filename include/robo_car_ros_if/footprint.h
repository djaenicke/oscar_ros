#ifndef ROBO_CAR_ROS_IF_FOOTPRINT_H
#define ROBO_CAR_ROS_IF_FOOTPRINT_H

#include "ros/ros.h"
#include <ros/console.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>

namespace robo_car_ros_if
{
class Footprint
{
 private:
  geometry_msgs::PolygonStamped polygon_msg_;
  geometry_msgs::Point32 point_;
  uint8_t num_points_;
  uint8_t point_idx_;

 public:
  Footprint(uint8_t num_points)
  {
    polygon_msg_.header.frame_id = "base_link";
    polygon_msg_.polygon.points.reserve(num_points);
    num_points_ = num_points;
    point_idx_ = 0;
    point_.x = 0.0f;
    point_.y = 0.0f;
    point_.z = 0.0f;

    for (uint8_t i = 0; i < num_points; i++)
    {
      polygon_msg_.polygon.points.push_back(point_);
    }
  }

  void AddPoint(float x, float y)
  {
    geometry_msgs::Point32 point;

    if (point_idx_ < num_points_)
    {
      polygon_msg_.polygon.points[point_idx_].x = x;
      polygon_msg_.polygon.points[point_idx_].y = y;
      point_idx_++;
    }
  }

  geometry_msgs::PolygonStamped GetPolyStampedMsg(void)
  {
    return polygon_msg_;
  }
};

}  // namespace robo_car_ros_if

#endif  // ROBO_CAR_ROS_IF_FOOTPRINT_H
