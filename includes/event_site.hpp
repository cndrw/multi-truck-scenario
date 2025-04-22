#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

struct Street
{
    int width;
    geometry_msgs::msg::Point direction; // direction of the street relative to event site
};

struct EventSite
{
    geometry_msgs::msg::Point position;
    std::vector<Street> streets;
    int width, height; // dimension of the event site
};
