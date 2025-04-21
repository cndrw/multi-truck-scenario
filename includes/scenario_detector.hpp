#pragma once

#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "multi_truck_scenario/msg/vehicle_base_data.hpp"


namespace mts_msgs = multi_truck_scenario::msg;


enum class Scenario
{
    None = 0,
    S1,
    S2
};

struct FValue
{
    float first;
    float second;
    float third;
};

class ScenarioDetector
{
public:
    ScenarioDetector();
    Scenario check(const std::vector<mts_msgs::VehicleBaseData>& vehicle);
    void set_implemenation(const int);

private:
    Scenario check_2(const std::vector<mts_msgs::VehicleBaseData>& vehicle); // impl 2 from T3100 
    Scenario check_1(const std::vector<mts_msgs::VehicleBaseData>& vehicle); // impl 2 from T3100
    std::vector<std::tuple<mts_msgs::VehicleBaseData, FValue>> apply_fuzzy_logic(const std::vector<mts_msgs::VehicleBaseData>&);
    FValue velocity_fuzzy_func(float velocity);
    float velocity_standing(float);
    float velocity_slow(float);
    float velocity_fast(float);
    FValue distance_fuzzy_func(float distance);
    float distance_inside(float);
    float distance_close(float);
    float distance_far(float);
    float get_distance_event_site(const geometry_msgs::msg::PointStamped& position) const;
    FValue apply_fuzzy_rules(const FValue& speed, const FValue& distance) const;

private:
    int m_implementation = -1;
    std::array<std::function<Scenario(const std::vector<mts_msgs::VehicleBaseData>&)>, 2> impl;
    rclcpp::Logger m_logger;
};