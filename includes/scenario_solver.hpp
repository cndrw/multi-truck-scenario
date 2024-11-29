#pragma once

#include <memory>

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

class ScenarioSolver
{
public:
    std::unique_ptr<SolutionType> solve(Scenario scenario, const std::vector<mts_msgs::VehicleBaseData::SharedPtr>& vehicles);
    void set_owner(int vin);


private:
    void solve_s1();
    void solve_s2();
    void pick_random_vehicle();
    void solve_right_of_way();
    int solve_S1_vorfahrt();

    // geometry util functions
    geometry_msgs::msg::PointStamped substract(const geometry_msgs::msg::PointStamped& p1, const geometry_msgs::msg::PointStamped& p2);

    std::vector<mts_msgs::VehicleBaseData::SharedPtr> m_vehicles;
    SolutionType m_solution;
    int m_owner_vin;
};


