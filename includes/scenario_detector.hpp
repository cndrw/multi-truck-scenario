#pragma once

#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "scenario_type.hpp"
#include "classification.hpp"


namespace mts_msgs = multi_truck_scenario::msg;


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
    void set_implemenation(const int detector, [[maybe_unused]] const int decision_algo /*impl 2 only*/);
    void set_owner(const int owner_vin);

private:
    Scenario check_2(const std::vector<mts_msgs::VehicleBaseData>& vehicle); // impl 2 from T3100 
    Scenario check_1(const std::vector<mts_msgs::VehicleBaseData>& vehicle); // impl 2 from T3100
    int get_event_site(const mts_msgs::VehicleBaseData& vehicle);
    std::vector<std::tuple<mts_msgs::VehicleBaseData, FValue>> apply_fuzzy_logic(const std::vector<mts_msgs::VehicleBaseData>&);
    FValue velocity_fuzzy_func(float velocity);
    float velocity_standing(float);
    float velocity_slow(float);
    float velocity_fast(float);
    FValue distance_fuzzy_func(float distance);
    float distance_inside(float);
    float distance_close(float);
    float distance_far(float);
    float get_event_site_distance(const geometry_msgs::msg::PointStamped& position) const;
    FValue apply_fuzzy_rules(const FValue& speed, const FValue& distance) const;
    /// @brief fuzzyfies the speed value and the distance to next "event site" -> sorts according to involement -> returns 
    /// @param vehicles vehicles to fuzzyfie 
    /// @return sorted list of vehicles according to involvement
    std::vector<mts_msgs::VehicleBaseData> get_sorted_vehicles(const std::vector<mts_msgs::VehicleBaseData>&);
    Scenario scenario_classification(const std::vector<mts_msgs::VehicleBaseData>& vehicles);
    Scenario decision_tree(const std::vector<mts_msgs::VehicleBaseData>& vehicles) const;
    void init_decision_tree();

private:
    int m_owner_vin = -1;
    int m_implementation = -1;
    int m_decision_algo = 0;
    int m_cur_event_site_id = -1;
    std::array<std::function<Scenario(const std::vector<mts_msgs::VehicleBaseData>&)>, 2> impl;
    std::array<std::function<Scenario(const std::vector<mts_msgs::VehicleBaseData>&)>, 2> m_decision_algo_impl;
    std::shared_ptr<cf::TreeNode<std::vector<mts_msgs::VehicleBaseData>>> m_dtree;
    rclcpp::Logger m_logger;
};