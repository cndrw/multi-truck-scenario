#pragma once

#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "multi_truck_scenario/msg/vehicle_base_data.hpp"


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
    Scenario check();
    void set_implemenation(const int);

private:
    Scenario check_2(); // impl 2 from T3100 
    Scenario check_1(); // impl 2 from T3100
    FValue velocity_fuzzy_func(float velocity);
    float velocity_standing(float);
    float velocity_slow(float);
    float velocity_fast(float);
    FValue distance_fuzzy_func(float distance);
    float distance_inside(float);
    float distance_close(float);
    float distance_far(float);
    float get_distance_event_site(const multi_truck_scenario::msg::VehicleBaseData& vehicle);

private:
    int m_implementation = -1;
    std::array<std::function<Scenario()>, 2> impl;
};