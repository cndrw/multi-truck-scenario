#include <iostream>
#include <chrono>
#include <memory>

#include "scenario_solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tutils.h"

static constexpr uint8_t S1_MAX_VEHICLES { 3 };
static constexpr uint8_t S2_MAX_VEHICLES { 4 };


void ScenarioSolver::set_owner(int vin)
{
    m_owner_vin = vin;
    m_solution.author_vin = vin;
}

std::unique_ptr<SolutionType> ScenarioSolver::solve(Scenario scenario, const std::vector<mts_msgs::VehicleBaseData>& vehicles)
{
    // reset solution 
    m_solution.winner_vin = VinFlags::Invalid;

    m_vehicles = vehicles;
    switch (scenario)
    {
        case Scenario::S1:
            solve_s1();
            break;

        case Scenario::S2:
            solve_s2();
            break;

        default:
            RCLCPP_ERROR(m_logger, "Scenario solution not implemented yet");
            break;
    }

    if (m_solution.winner_vin == VinFlags::Invalid)
    {
        return nullptr;
    }
    else 
    {
        return std::make_unique<SolutionType>(m_solution);
    }
}

ScenarioSolver::ScenarioSolver() : m_logger(rclcpp::get_logger("solver"))
{
}

void ScenarioSolver::solve_s1()
{
    // get the vehicle that would have had right to drive
    int priority_car_vin = solve_uncontrolled_intersection();

    // get vehicle data from vin
    const auto vehicle = *std::find_if(m_vehicles.begin(), m_vehicles.end(), [priority_car_vin](const auto& v){
        return v.vin == priority_car_vin;
    });

    int upgraded_priority_car = get_vehicle(vehicle, Side::left);

    m_solution.author_vin = m_owner_vin;
    m_solution.winner_vin = upgraded_priority_car;
}

void ScenarioSolver::solve_s2()
{
    if (m_vehicles.size() == S2_MAX_VEHICLES)
    {
        pick_random_vehicle();
    }
    else
    {
        const int winner = solve_uncontrolled_intersection();
        m_solution.winner_vin = winner;
    }
}

int ScenarioSolver::get_vehicle(const mts_msgs::VehicleBaseData& vehicle, Side side)
{
    int winner_vin = VinFlags::Invalid;
    constexpr auto reference_angle = 90.0;
    const auto adjust_angle = reference_angle - vehicle.direction;

    for (const auto& other : m_vehicles)
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
        if (side == Side::left)
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

bool ScenarioSolver::is_opposite(float alpha, float beta) const
{
    return std::abs(std::abs(alpha - beta) - 180) < 10;
}

int ScenarioSolver::solve_uncontrolled_intersection()
{
    for (const auto& vehicle: m_vehicles)
    {
        if (get_vehicle(vehicle, Side::right) == VinFlags::Invalid)
        {
            m_solution.author_vin = m_owner_vin;
            return vehicle.vin;
        }
    }
    return VinFlags::Invalid;
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
    else
    {
        m_solution.winner_vin = VinFlags::Ignored;
    }
}
