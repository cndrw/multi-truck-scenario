#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "scenario_detector.hpp"

#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "multi_truck_scenario/msg/s2_solution.hpp"


namespace mts_msgs = multi_truck_scenario::msg;

// general purpose solution type 
// based on scenario only some elemnts will be needed
struct SolutionType
{
    int author_vin;
    int winner_vin;
};

enum class PriorityRule
{
    right,
    left
};

class ScenarioSolver
{
public:
    ScenarioSolver();
    std::unique_ptr<SolutionType> solve(Scenario scenario, const std::vector<mts_msgs::VehicleBaseData::SharedPtr>& vehicles);
    void set_owner(int vin);

private:
    rclcpp::Logger m_logger;


private:
    void solve_s1();
    void solve_s2();
    int solve_uncontrolled_intersection(PriorityRule rule);
    int get_vehicle_right(int vin);
    int get_vehicle_left(int vin);
    bool is_opposite(float alpha, float beta) const;
    void pick_random_vehicle();

    // geometry util functions
    geometry_msgs::msg::PointStamped substract(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2);

    std::vector<mts_msgs::VehicleBaseData::SharedPtr> m_vehicles;
    SolutionType m_solution;
    int m_owner_vin;
};


