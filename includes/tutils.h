#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

// truck-utils
namespace tutils {
    static constexpr auto RAD2DEG { 180 / M_PI };
    static constexpr auto DEG2RAD { M_PI / 180 };

    geometry_msgs::msg::PointStamped substract(
        const geometry_msgs::msg::PointStamped& p1,
        const geometry_msgs::msg::PointStamped& p2
    );

    geometry_msgs::msg::PointStamped add(
        const geometry_msgs::msg::PointStamped& p1,
        const geometry_msgs::msg::PointStamped& p2
    );

    geometry_msgs::msg::Point add(
        const geometry_msgs::msg::Point& p1,
        const geometry_msgs::msg::Point& p2
    );

    geometry_msgs::msg::Point add(
        const geometry_msgs::msg::Point& p1,
        const double factor
    );
}
