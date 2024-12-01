#include <iostream>
#include <chrono>
#include <memory>

#include "scenario_solver.hpp"
#include "rclcpp/rclcpp.hpp"

static constexpr uint8_t S1_MAX_VEHICLES { 3 };
static constexpr uint8_t S2_MAX_VEHICLES { 4 };
static constexpr auto RAD2DEG { 180 / M_PI };
static constexpr auto INVALID_VIN { -1 };

void ScenarioSolver::set_owner(int vin)
{
    m_owner_vin = vin;
    m_solution.author_vin = vin;
}

std::unique_ptr<SolutionType> ScenarioSolver::solve(Scenario scenario, const std::vector<mts_msgs::VehicleBaseData::SharedPtr>& vehicles)
{
    // reset solution 
    m_solution.winner_vin = INVALID_VIN;

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

ScenarioSolver::ScenarioSolver() : m_logger(rclcpp::get_logger("solver"))
{
}

void ScenarioSolver::solve_s1()
{
    // get the car that would have had right to drive
    int priority_car_vin = solve_uncontrolled_intersection(PriorityRule::right);

    /* 
    const auto priority_car = std::find_if(m_vehicles.begin(), m_vehicles.end(), [priority_car_vin](const auto& v){
        return v->vin == priority_car_vin;
    });
    */

   int upgraded_priority_car = get_vehicle_left(priority_car_vin);
   RCLCPP_INFO(m_logger, "winner %d", upgraded_priority_car);
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
        const int winner = solve_uncontrolled_intersection(PriorityRule::right);
        m_solution.winner_vin = winner;
    }
}

int ScenarioSolver::get_vehicle_left(const int vin)
{
    const auto vehicle = std::find_if(m_vehicles.begin(), m_vehicles.end(), [vin](const auto& v){
        return v->vin == vin;
    })[0];

    const auto adjust_angle = 90 - vehicle->direction;
    const auto adjusted_v1_angle = vehicle->direction + adjust_angle; 
    int winner_vin = INVALID_VIN;

    for (const auto& other : m_vehicles)
    {
        if (other == vehicle) continue;

        if (is_opposite(vehicle->direction, other->direction))
        {
            continue;
        }

        // get direction vector from v1 to v2
        const auto diff = substract(vehicle->position, other->position);
        
        // angle of the direction vector
        auto diff_angle = std::atan2(diff.point.y, diff.point.x) * RAD2DEG;

        diff_angle += adjust_angle; // also adjust the angle of the differenz vector

        if (diff_angle < 0)
        {
            diff_angle += 360;
        }

        if (diff_angle >= adjusted_v1_angle)
        {
            winner_vin = other->vin;
        }
    }

    return winner_vin;
}

bool ScenarioSolver::is_opposite(float alpha, float beta) const
{
    return std::abs(std::abs(alpha - beta) - 180) < 10;
}

int ScenarioSolver::get_vehicle_right(const int vin)
{

}

int ScenarioSolver::solve_uncontrolled_intersection(PriorityRule rule)
{
    int winner_vin = INVALID_VIN; // TODO: should actually no be ignored if that comes throu
    for (const auto& v1 : m_vehicles)
    {
        size_t count = 0;
        // get the adjustment value so that v1.direction - adjustment_value equals 90
        const auto adjust_angle = 90 - v1->direction;
        const auto adjusted_v1_angle = v1->direction + adjust_angle; 

        for (const auto& v2 : m_vehicles)
        {
            if (v1 == v2) continue;

            // get direction vector from v1 to v2
            const auto diff = substract(v1->position, v2->position);
            
            // angle of the direction vector
            auto diff_angle = std::atan2(diff.point.y, diff.point.x) * RAD2DEG;

            diff_angle += adjust_angle; // also adjust the angle of the differenz vector

            if (diff_angle < 0)
            {
                diff_angle += 360;
            }

            // test if vehicles are opposite
            // RCLCPP_INFO(m_logger, "v1: %f, v2: %f, diff %f)", v1->direction, v2->direction, std::abs(v1->direction - v2->direction));
            if (is_opposite(v1->direction, v2->direction))
            {
                count++;
                RCLCPP_INFO(m_logger, "[%d] car %d +1 (%d)", m_owner_vin, v1->vin, v2->vin);
                continue;
            }

            // if differenz angle is creater than the one of the vehicle under test (VUT) -> it is to the left of VUT 
            bool result = diff_angle <= adjusted_v1_angle;
            if (rule == PriorityRule::left)
            {
                result = !result;
            }

            // if a vehicle is to the right -> no right to drive
            if (result)
            {
                RCLCPP_INFO(m_logger, "car %d is to the right of %d", v2->vin, v1->vin);
                break;
            }
            else
            {
                RCLCPP_INFO(m_logger, "[%d] car %d +1 (%d)", m_owner_vin, v1->vin, v2->vin);
                count++;
            }
        }

        // if all cars are not "right" than this one has to drive
        if (count == m_vehicles.size() - 1)
        {
            winner_vin = v1->vin;
        }
    } 

    m_solution.author_vin = m_owner_vin;
    return winner_vin;
}

void ScenarioSolver::pick_random_vehicle()
{
    bool is_smallest_vin = std::all_of(m_vehicles.begin(), m_vehicles.end(), [this](const auto v){
        return m_owner_vin <= v->vin;  
    });

    if (is_smallest_vin)
    {
        std::srand(std::time(0)); 
        int rnd_vin = (std::rand() % 4) + 1; 

        m_solution.author_vin = m_owner_vin;
        m_solution.winner_vin = rnd_vin;
    }
}

/*
 * result: p2 - p1
 */
geometry_msgs::msg::PointStamped ScenarioSolver::substract(
    const geometry_msgs::msg::PointStamped& p1,
    const geometry_msgs::msg::PointStamped& p2
)
{
    auto tmp = geometry_msgs::msg::PointStamped();
    tmp.point.x = p2.point.x - p1.point.x;
    tmp.point.y = p2.point.y - p1.point.y;
    return tmp;
}
