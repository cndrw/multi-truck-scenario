#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "event_site.hpp"

enum VinFlags
{
    None = 0,
    Invalid = -1,
    Ignored = -2
};

enum class Indicator { Off, Left, Right, Warning };
enum class Side { Right, Left };

// truck-utils
namespace tutils {
    namespace mts_msgs = multi_truck_scenario::msg;

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

    geometry_msgs::msg::Point add(
        const geometry_msgs::msg::Point& p1,
        const double x,
        const double y,
        const double z
    );

    geometry_msgs::msg::Point multiply(
        const geometry_msgs::msg::Point& p1,
        const double factor
    );

    // return the given angle as angle between 0-360
    double norm_angle(double angle);


    int get_vehicle(const std::vector<mts_msgs::VehicleBaseData>& vehicles, const mts_msgs::VehicleBaseData& vehicle, Side side);
    bool is_opposite(float alpha, float beta);
    int get_street(const mts_msgs::VehicleBaseData& vehicle);
    int get_street(int street_id, Side side);
}
