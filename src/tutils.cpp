#include "tutils.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace tutils {

geometry_msgs::msg::PointStamped substract(
    const geometry_msgs::msg::PointStamped& p1,
    const geometry_msgs::msg::PointStamped& p2
) 
{
    auto tmp = geometry_msgs::msg::PointStamped();
    tmp.point.x = p2.point.x - p1.point.x;
    tmp.point.y = p2.point.y - p1.point.y;
    return tmp;
}

geometry_msgs::msg::PointStamped add(
    const geometry_msgs::msg::PointStamped& p1,
    const geometry_msgs::msg::PointStamped& p2
) 
{
    auto p = geometry_msgs::msg::PointStamped();
    p.point.x = p1.point.x + p2.point.x;
    p.point.y = p1.point.y + p2.point.y;
    return p;
}

}
