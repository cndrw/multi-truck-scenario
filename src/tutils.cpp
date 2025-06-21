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

geometry_msgs::msg::Point add(
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2
)
{
    auto p = geometry_msgs::msg::Point();
    p.x = p1.x + p2.x;
    p.y = p1.y + p2.y;
    p.z = p1.z + p2.z;
    return p;
}
geometry_msgs::msg::Point add(const geometry_msgs::msg::Point &p1, const double factor)
{
    auto p = geometry_msgs::msg::Point();
    p.x = p1.x + factor;
    p.y = p1.y + factor;
    p.z = p1.z + factor;
    return p;
}

geometry_msgs::msg::Point add(const geometry_msgs::msg::Point &p1, const double x, const double y, const double z)
{
    auto p = geometry_msgs::msg::Point();
    p.x = p1.x + x;
    p.y = p1.y + y;
    p.z = p1.z + z;
    return p;
}

geometry_msgs::msg::Point multiply(const geometry_msgs::msg::Point &p1, const double factor)
{
    auto p = geometry_msgs::msg::Point();
    p.x = p1.x * factor;
    p.y = p1.y * factor;
    p.z = p1.z * factor;
    return p;
}

geometry_msgs::msg::Point create_point(const double x, const double y, const double z)
{
    auto p = geometry_msgs::msg::Point();
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

double norm_angle(double angle)
{
    double result = std::fmod(angle, 360.0);
    if (result < 0) result += 360.0; // für negative Werte absichern
    return result;

}

int get_vehicle(const std::vector<mts_msgs::VehicleBaseData>& vehicles, const mts_msgs::VehicleBaseData& vehicle, Side side)
{
    int winner_vin = VinFlags::Invalid;
    constexpr auto reference_angle = 90.0;
    const auto adjust_angle = reference_angle - vehicle.direction;

    for (const auto& other : vehicles)
    {
        if (other == vehicle) continue;

        if (is_opposite(vehicle.direction, other.direction))
        {
            continue;
        }

        // get direction vector from v1 to v2
        const auto diff = tutils::substract(vehicle.position, other.position);
        
        // angle of the direction vector
        auto diff_angle = std::atan2(diff.point.y, diff.point.x) * tutils::RAD2DEG;

        diff_angle += adjust_angle; // also adjust the angle of the differenz vector

        if (diff_angle < 0)
        {
            diff_angle += 360;
        }

        if (diff_angle >= 360)
        {
            diff_angle -= 360;
        }

        // determine on which side it is
        bool result = diff_angle <= reference_angle;
        if (side == Side::Left)
        {
            result = !result;
        }

        if (result)
        {
            winner_vin = other.vin;
        }
    }

    return winner_vin;

}

bool is_opposite(float alpha, float beta)
{
    return std::abs(std::abs(alpha - beta) - 180) < 10;
}

int get_street(const mts_msgs::VehicleBaseData &vehicle)
{
    // temporary solution

    int street = -1;
    switch ((int)vehicle.direction)
    {
        case 0 || 360: street = 1; break;
        case 90: street = 3; break;
        case 180: street = 0; break;
        case 270: street = 2; break;
    }
    return street;
}

int get_street(const int street_id, const Side side)
{
    // quick 'n dirty temporary solution
    if (side == Side::Right)
    {
        switch (street_id)
        {
            case 0: return 3;
            case 1: return 2;
            case 2: return 0;
            case 3: return 1;
        }
    }

    if (side == Side::Left)
    {
        switch (street_id)
        {
            case 0: return 2;
            case 1: return 3;
            case 2: return 1;
            case 3: return 0;
        }
    }

    return -1;
}

}
