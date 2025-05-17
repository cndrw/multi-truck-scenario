#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

struct Street
{
    int width;
};

struct EventSite
{
    geometry_msgs::msg::Point position;
    std::vector<Street> streets;
    /* Width values of streets around the event site
       always 4 values for left, right, top, bottom
       const order: [left, right, top, bottom] */
    int width, height; // dimension of the event site
};
