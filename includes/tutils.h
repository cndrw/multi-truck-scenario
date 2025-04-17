#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

// truck-utils
namespace tutils {

    geometry_msgs::msg::PointStamped substract(
        const geometry_msgs::msg::PointStamped& p1,
        const geometry_msgs::msg::PointStamped& p2
    );

    geometry_msgs::msg::PointStamped add(
        const geometry_msgs::msg::PointStamped& p1,
        const geometry_msgs::msg::PointStamped& p2
    );
}
