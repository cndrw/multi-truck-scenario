#include <iostream>
#include <chrono>
#include <memory>

#include "scenario_solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tutils.h"

static constexpr uint8_t S2_MAX_VEHICLES { 4 };
static constexpr auto RAD2DEG { 180 / M_PI };
static constexpr auto INVALID_VIN { -1 };

void ScenarioSolver::set_owner(int vin)
{
    m_owner_vin = vin;
    m_solution.author_vin = vin;
}

std::unique_ptr<SolutionType> ScenarioSolver::solve(Scenario scenario, const std::vector<mts_msgs::VehicleBaseData>& vehicles)
{
    // reset solution 
    m_solution.winner_vin = INVALID_VIN;

    m_vehicles = vehicles;
    switch (scenario)
    {
        case Scenario::S2:
            solve_s2();
            break;

        default:
            std::cerr << "ERROR: Scenario solution not implemented yet\n"; 
            break;
    }

    if (m_solution.winner_vin == INVALID_VIN)
    {
        return nullptr;
    }
    else 
    {
        return std::make_unique<SolutionType>(m_solution);
    }
}

void ScenarioSolver::solve_s2()
{
    if (m_vehicles.size() == S2_MAX_VEHICLES)
    {
        pick_random_vehicle();
    }
    else
    {
        solve_right_of_way();
    }
}

void ScenarioSolver::pick_random_vehicle()
{
    bool is_smallest_vin = std::all_of(m_vehicles.begin(), m_vehicles.end(), [this](const auto v){
        return m_owner_vin <= v.vin;  
    });

    if (is_smallest_vin)
    {
        std::srand(std::time(0)); 
        int rnd_vin = (std::rand() % 4) + 1; 

        m_solution.author_vin = m_owner_vin;
        m_solution.winner_vin = rnd_vin;
    }
}

void ScenarioSolver::solve_right_of_way()
{
    int winner_vin = INVALID_VIN; // TODO: should actually no be ignored if that comes throu
    for (const auto& v1 : m_vehicles)
    {
        size_t count = 0;
        // get the adjustment value so that v1.direction - adjustment_value equals 90
        const auto adjust_angle = 90 - v1.direction;
        const auto adjusted_v1_angle= v1.direction + adjust_angle; 

        for (const auto& v2 : m_vehicles)
        {
            if (v1 == v2) continue;

            const auto diff = tutils::substract(v1.position, v2.position);
            
            auto diff_angle = std::atan2(diff.point.y, diff.point.x) * RAD2DEG;
            diff_angle += adjust_angle; // also adjust the angle of the differenz vector

            if (diff_angle < 0)
            {
                diff_angle += 360;
            }

            if (diff_angle >= adjusted_v1_angle)
            {
                count++;
            }
        }

            // if all cars are not "right" than this one has to drive
        if (count == m_vehicles.size() - 1)
        {
            winner_vin = v1.vin;
        }
    } 

    m_solution.author_vin = m_owner_vin;
    m_solution.winner_vin = winner_vin;
}

