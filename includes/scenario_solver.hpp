#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "scenario_detector.hpp"

#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "multi_truck_scenario/msg/solution.hpp"


namespace mts_msgs = multi_truck_scenario::msg;

// general purpose solution type 
// based on scenario only some elemnts will be needed
struct SolutionType
{
    int author_vin;
    int winner_vin;
};

enum class Side
{
    right,
    left
};

class ScenarioSolver
{
public:
    ScenarioSolver();
    std::unique_ptr<SolutionType> solve(Scenario scenario, const std::vector<mts_msgs::VehicleBaseData>& vehicles);
    // std::unique_ptr<SolutionType> solve(Scenario scenario, const std::vector<mts_msgs::VehicleBaseData::SharedPtr>& vehicles);
    void set_owner(int vin);

private:
    rclcpp::Logger m_logger;


private:
    void solve_s1();
    void solve_s2();
    int solve_uncontrolled_intersection();
    int get_vehicle(const mts_msgs::VehicleBaseData& vehicle, Side side);
    bool is_opposite(float alpha, float beta) const;
    void pick_random_vehicle();

    std::vector<mts_msgs::VehicleBaseData> m_vehicles;
    SolutionType m_solution;
    int m_owner_vin;
};


