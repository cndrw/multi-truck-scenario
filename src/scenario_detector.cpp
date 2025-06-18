/* TODOs:
*   - kleinen entscheidungsbaum implementieren
*/

#include <iostream>
#include <vector>

#include "scenario_detector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "multi_truck_scenario/srv/get_event_site_distance.hpp"
#include "multi_truck_scenario/srv/get_event_site_id.hpp"
#include "multi_truck_scenario/msg/event_site_data.hpp"
#include "tutils.h"
#include "classification.hpp"

using namespace std::chrono_literals;
namespace mts_msgs = multi_truck_scenario::msg;
namespace mts_srvs = multi_truck_scenario::srv;


void ScenarioDetector::set_implemenation(const int detector, [[maybe_unused]] const int decision_algo /*impl 2 only*/)
{
    m_implementation = detector;
    m_decision_algo = decision_algo; 
    RCLCPP_INFO(m_logger, "set: %d, %d", detector, decision_algo);
}

void ScenarioDetector::set_owner(const int owner_vin)
{
    m_owner_vin = owner_vin;
}

ScenarioDetector::ScenarioDetector() : m_logger(rclcpp::get_logger("detector"))
{
    using namespace std::placeholders;
    impl[0] = std::bind(&ScenarioDetector::check_1, this, _1);
    impl[1] = std::bind(&ScenarioDetector::check_2, this, _1);

    m_decision_algo_impl[0] = std::bind(&ScenarioDetector::decision_tree, this, _1);
    m_decision_algo_impl[1] = std::bind(&ScenarioDetector::knn_classify, this, _1);

    init_decision_tree();
    init_knn();
    init_bayesian_network();
}

std::pair<Scenario, std::vector<mts_msgs::VehicleBaseData>>
ScenarioDetector::check(const std::vector<mts_msgs::VehicleBaseData>& vehicles)
{
    if (vehicles.empty() || vehicles.size() == 1)
    {
        return {Scenario::None, vehicles};
    }
    return {impl[m_implementation](vehicles), m_vehicles};
}

Scenario ScenarioDetector::check_1([[maybe_unused]] const std::vector<mts_msgs::VehicleBaseData>& vehicles)
{
    // get nearest event site to owner vin
    auto owner_vehicle = *std::find_if(vehicles.begin(), vehicles.end(), [this](const auto& v) {
        return v.vin == this->m_owner_vin;
    });

    const auto event_site = get_event_site(owner_vehicle);
    const auto& site_id = event_site.first;
    const auto& site_data = event_site.second;

    std::vector<Scenario> possible_scenario;

    for (const auto& vehicle : vehicles)
    {
        // distance
        const auto dist = get_event_site_distance(vehicle.position, site_id);
        const FValue fdist = distance_fuzzy_func(dist);

        // street width
        const bool has_small_street = std::any_of(site_data.streets.begin(), site_data.streets.end(), [](const mts_msgs::StreetData s){
            return s.width < 2;
        });

        possible_scenario.push_back(evalute_bayesian(vehicles.size(), has_small_street, fdist));
    }

    // determine which scneario is most often in the vector
    std::unordered_map<Scenario, int> freq;
    for (Scenario num : possible_scenario)
    {
        freq[num]++;
    }

    const auto scenario_result = std::max_element(freq.begin(), freq.end(), [](const auto& a, const auto& b) {
        return a.second < b.second;
    })->first;

    RCLCPP_INFO(m_logger, "Vehicle %d detected scenario: %d", m_owner_vin, scenario_result);

    return scenario_result;
}

Scenario ScenarioDetector::evalute_bayesian(const int num_vehicle, const bool has_small_street, const FValue distance)
{
    int num_vehicle_i = 0; // [0 1 2]
    switch (num_vehicle)
    {
        case 2: num_vehicle_i = 1; break;
        case 3: num_vehicle_i = 1; break;
        case 4: num_vehicle_i = 2; break;
    }

    int streets_i = static_cast<int>(!has_small_street);
    
    int distance_i = 1; // [0 1 2]
    const auto max_val = std::max({distance.first, distance.second, distance.third});
    if (distance.first == max_val) distance_i = 0;
    else if (distance.second == max_val) distance_i = 1;
    else if (distance.third == max_val) distance_i = 2;

    int index = num_vehicle_i * 18 + streets_i * 9 + distance_i * 3;

    auto prob_s1 = m_bayesian_net[index];
    auto prob_s2 = m_bayesian_net[index + 1];
    auto prob_none = m_bayesian_net[index + 2];

    const auto max_prob = std::max({ prob_s1, prob_s2, prob_none });
    Scenario res = Scenario::None;
    if (max_prob == prob_s1) res = Scenario::S1;
    if (max_prob == prob_s2) res = Scenario::S2;
    if (max_prob == prob_none) res = Scenario::None;
    return res;
}

void ScenarioDetector::init_bayesian_network()
{
    m_bayesian_net = {
    // Anzahl Fzg. | Straße | Position |   S1   |   S2   | Nichts
    /* <=1         | schmal | nahe */   0.2364 , 0.1182 , 0.6454,
    /* <=1,        | schmal | drin */   0.1195 , 0.1195 , 0.761,
    /* <=1,        | schmal | fern */   0.103  , 0.0772 , 0.8198,
    /* <=1,        | breit  | nahe */   0.0245 , 0.1718 , 0.8037,
    /* <=1,        | breit  | drin */   0.0109 , 0.1531 , 0.8359,
    /* <=1,        | breit  | fern */   0.0093 , 0.0981 , 0.8926,
    /*(1-3],       | schmal | nahe */   0.7667 , 0.2236 , 0.0097,
    /*(1-3],       | schmal | drin */   0.62   , 0.3617 , 0.0183,
    /*(1-3],       | schmal | fern */   0.6783 , 0.2967 , 0.025,
    /*(1-3],       | breit  | nahe */   0.191  , 0.78   , 0.029,
    /*(1-3],       | breit  | drin */   0.0962 , 0.7511 , 0.1527,
    /*(1-3],       | breit  | fern */   0.0864 , 0.5643 , 0.3493,
    /* >=4,        | schmal | nahe */   0.5    , 0.5    , 0,
    /* >=4,        | schmal | drin */   0.3333 , 0.6667 , 0,
    /* >=4,        | schmal | fern */   0.4    , 0.6    , 0,
    /* >=4,        | breit  | nahe */   0.0667 , 0.9333 , 0,
    /* >=4,        | breit  | drin */   0.0333 , 0.9667 , 0,
    /* >=4,        | breit  | fern */   0.03   , 0.97   , 0
    };
}

std::pair<int, mts_msgs::EventSiteData> ScenarioDetector::get_event_site(const mts_msgs::VehicleBaseData &vehicle)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("map_requester");
    rclcpp::Client<mts_srvs::GetEventSiteID>::SharedPtr client =
        node->create_client<mts_srvs::GetEventSiteID>("get_event_site_id");

    auto request = std::make_shared<mts_srvs::GetEventSiteID::Request>();
    request->position = vehicle.position;

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            break;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        return {result.get()->id, result.get()->event_site};
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_event_site");
        return std::pair<int, mts_msgs::EventSiteData>{};
    }
}

std::vector<std::tuple<mts_msgs::VehicleBaseData, FValue>> 
ScenarioDetector::apply_fuzzy_logic(const std::vector<mts_msgs::VehicleBaseData>& vehicles, const int id) 
{
    std::vector<std::tuple<mts_msgs::VehicleBaseData, FValue>> fuzzy_vehicles;
    for (const auto& vehicle : vehicles)
    {
        FValue velocity = velocity_fuzzy_func(vehicle.speed); 
        FValue distance = distance_fuzzy_func(get_event_site_distance(vehicle.position, id));
        // apply fuzzy rules to involveed_vehicles (according to their involvement B)
        FValue involvement = apply_fuzzy_rules(velocity, distance);

        // RCLCPP_INFO(m_logger, "(%d von %d) vel 1: %f  2: %f  3: %f", vehicle.vin, m_owner_vin, velocity.first, velocity.second, velocity.third);
        // RCLCPP_INFO(m_logger, "(%d von %d) dist 1: %f  2: %f  3: %f", vehicle.vin, m_owner_vin, distance.first, distance.second, distance.third);
        // RCLCPP_INFO(m_logger, "(%d von %d) involvement 1: %f  2: %f  3: %f", vehicle.vin, m_owner_vin, involvement.first, involvement.second, involvement.third);

        if (involvement.second > involvement.first)
        {
            continue;
        }

        fuzzy_vehicles.push_back({vehicle, involvement});
    }
    return fuzzy_vehicles;
}

Scenario ScenarioDetector::check_2(const std::vector<mts_msgs::VehicleBaseData>& vehicles)
{
    m_vehicles.clear();
    std::vector<mts_msgs::VehicleBaseData>  invoveld_vehicles;

    // get nearest event site to owner vin
    auto owner_vehicle = *std::find_if(vehicles.begin(), vehicles.end(), [this](const auto& v) {
        return v.vin == this->m_owner_vin;
    });

    const auto event_site = get_event_site(owner_vehicle);
    const auto& site_id = event_site.first;
    const auto& site_data = event_site.second;

    // filter out vehicles that are facing/driving away from the event site
    for (const auto& vehicle : vehicles)
    {
        // d(r) <= d(r + a)
        auto dir = geometry_msgs::msg::PointStamped();
        dir.point.x = std::cos(vehicle.direction * tutils::DEG2RAD);
        dir.point.y = std::sin(vehicle.direction * tutils::DEG2RAD);

        if (
            get_event_site_distance(vehicle.position, site_id) >=
            get_event_site_distance(tutils::add(vehicle.position, dir), site_id))
        {
            invoveld_vehicles.push_back(vehicle);
        }
        else if (vehicle.vin == m_owner_vin)
        {
            RCLCPP_INFO(m_logger, "Vehicle %d is not involved in viewed scenario", m_owner_vin);
            return Scenario::None;
        }
    }

    auto sorted_vehicles = get_sorted_vehicles(invoveld_vehicles, site_id);

    // check if owner vehicle is still relevant for scenario
    auto owner_v = std::find_if(vehicles.begin(), vehicles.end(), [this](const auto& v) {
        return v.vin == this->m_owner_vin;
    });

    if (sorted_vehicles.empty() || sorted_vehicles.size() == 1 || owner_v->vin != m_owner_vin)
    {
        RCLCPP_INFO(m_logger, "(%d) Invalid amount of involved Vehicles in viewed Scenario (%d)", m_owner_vin, sorted_vehicles.size());
        return Scenario::None;
    }

    Scenario scenario_result = Scenario::None; 
    for (size_t i = 2; i <= sorted_vehicles.size(); i++)
    {
        std::vector<mts_msgs::VehicleBaseData> sub(sorted_vehicles.begin(), sorted_vehicles.begin() + i);
        const auto cur_scenario = scenario_classification({sub, site_data});

        if (cur_scenario != Scenario::None)
        {
            scenario_result = cur_scenario;
        }
        else 
        {
            break;
        }
    }

    m_vehicles = sorted_vehicles; // involved vehicles (final)
    RCLCPP_INFO(m_logger, "Vehicle %d detected scenario: %d", m_owner_vin, scenario_result);

    return scenario_result;
}


FValue ScenarioDetector::velocity_fuzzy_func(float velocity)
{
    return
    {
        velocity_standing(velocity),
        velocity_slow(velocity),
        velocity_fast(velocity)
    };
}

float ScenarioDetector::velocity_standing(float velocity)
{
    if (velocity <= 2)
    {
        return 1.0;
    }

    if (velocity >= 0)
    {
        return 0.0;
    }

    return -1.0 / 3.0 * (velocity - 5.0);
}

float ScenarioDetector::velocity_slow(float velocity)
{
    if (velocity > 2 && velocity <= 7)
    {
        return 1.0 / 5.0 * (velocity - 2.0);
    }

    if (velocity > 7 && velocity <= 10 )
    {
        return 1.0;
    }

    if (velocity > 10 && velocity < 20)
    {
        return - 1.0 / 10.0 * (velocity - 20.0);
    }

    return 0.0; // velocity <= 2 || velocity >= 20
}

float ScenarioDetector::velocity_fast(float velocity)
{
    if (velocity <= 15)
    {
        return 0.0;
    }

    if (velocity > 15 && velocity < 30)
    {
        return 1.0 / 15.0 * (velocity - 15.0);
    }

    return 1.0; // velocity >= 30
}

FValue ScenarioDetector::distance_fuzzy_func(const float distance)
{
    return
    {
        distance_inside(distance),
        distance_close(distance),
        distance_far(distance)
    };
}

float ScenarioDetector::distance_inside(float distance)
{
    if (distance <= -2)
    {
        return 1.0;
    }

    if (distance > -2 && distance < 0)
    {
        return -distance / 2.0;
    }

    return 0.0; // distance >= 0
}

float ScenarioDetector::distance_close(float distance)
{
    if (distance > -1 && distance <= 0)
    {
        return distance + 1.0;
    }    

    if (distance > 0 && distance < 10)
    {
        return -distance / 10.0 + 1.0;
    }

    return 0.0; // distance <= -1 || distance >= 10
}

float ScenarioDetector::distance_far(float distance)
{
    if (distance > 5 && distance <= 15)
    {
        return 1.0/10.0 * (distance - 5.0);
    }

    if (distance > 15)
    {
        return 1.0;
    }

    return 0.0;
}

float ScenarioDetector::get_event_site_distance(const geometry_msgs::msg::PointStamped& position, const int id) const
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("map_requester");
    rclcpp::Client<mts_srvs::GetEventSiteDistance>::SharedPtr client =
        node->create_client<mts_srvs::GetEventSiteDistance>("get_event_site_distance");

    auto request = std::make_shared<mts_srvs::GetEventSiteDistance::Request>();
    request->position = position;
    request->event_site_id = id;

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            break;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        return result.get()->distance;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_event_site_distance");
        return 0;
    }
}

FValue ScenarioDetector::apply_fuzzy_rules(const FValue &speed, const FValue &distance) const
{
    return FValue
    {
        std::min(distance.second, std::max(speed.second, speed.first)),
        std::max(speed.third, distance.third),
        -1 
    };
}

std::vector<mts_msgs::VehicleBaseData> ScenarioDetector::get_sorted_vehicles(const std::vector<mts_msgs::VehicleBaseData>& vehicles, const int id)
{
    auto fuzzy_vehicles = apply_fuzzy_logic(vehicles, id);

    // sort vehicles according to involment "involved" fuzzy variable
    std::sort(fuzzy_vehicles.begin(), fuzzy_vehicles.end(), [](auto const& t1, auto const& t2) {
        return std::get<1>(t1).first < std::get<1>(t2).first;
    });

    std::vector<mts_msgs::VehicleBaseData> result(fuzzy_vehicles.size());

    // fuzzy variable no longer needed -> return sorted array of vehicles
    std::transform(fuzzy_vehicles.begin(), fuzzy_vehicles.end(), result.begin(),
        [](const auto& t) {
            return std::get<0>(t);
        }
    );

    return result;
}

Scenario ScenarioDetector::scenario_classification(const DecisionData& data)
{
    return m_decision_algo_impl[m_decision_algo](data);
}

void ScenarioDetector::init_decision_tree()
{
    m_dtree = std::make_shared<cf::TreeNode<DecisionData>>([](const DecisionData& data) {
        return data.second.num_streets > 2;
    });

    m_dtree->no = std::make_shared<cf::TreeNode<DecisionData>>(Scenario::None); // potential Scenario s3

    m_dtree->yes = std::make_shared<cf::TreeNode<DecisionData>>([](const DecisionData& data){
        return data.first.size() == 4;
    });

    m_dtree->yes->yes = std::make_shared<cf::TreeNode<DecisionData>>(Scenario::S2);
    m_dtree->yes->no = std::make_shared<cf::TreeNode<DecisionData>>([](const DecisionData& data){
        // priority vehicle does NOT indicator left
        const auto& vehicles = data.first;
        const auto& priority_vehicle = *std::find_if(vehicles.begin(), vehicles.end(), [&vehicles](const auto& v){
            return tutils::get_vehicle(vehicles, v, Side::Right) == VinFlags::Invalid;
        });

        return priority_vehicle.indicator_state != (int)Indicator::Left;
    });

    m_dtree->yes->no->yes = std::make_shared<cf::TreeNode<DecisionData>>(Scenario::S2);

    m_dtree->yes->no->no = std::make_shared<cf::TreeNode<DecisionData>>([](const DecisionData& data){
        // street left of priority vehicle is small?
        const auto& vehicles = data.first;
        const auto& priority_vehicle = *std::find_if(vehicles.begin(), vehicles.end(), [&vehicles](const auto& v){
            return tutils::get_vehicle(vehicles, v, Side::Right) == VinFlags::Invalid;
        });

        const auto street_id = tutils::get_street(priority_vehicle);
        const auto street_id_left = tutils::get_street(street_id, Side::Left);

        return data.second.streets[street_id_left].width < 2;
    });

    m_dtree->yes->no->no->no = std::make_shared<cf::TreeNode<DecisionData>>(Scenario::S2);

    m_dtree->yes->no->no->yes = std::make_shared<cf::TreeNode<DecisionData>>([](const DecisionData& data){
        // is a vehicle to the left of the priority vehicle?
        const auto& vehicles = data.first;
        const auto& priority_vehicle = *std::find_if(vehicles.begin(), vehicles.end(), [&vehicles](const auto& v){
            return tutils::get_vehicle(vehicles, v, Side::Right) == VinFlags::Invalid;
        });

        return tutils::get_vehicle(vehicles, priority_vehicle, Side::Left) != VinFlags::Invalid;
    });

    m_dtree->yes->no->no->yes->no = std::make_shared<cf::TreeNode<DecisionData>>(Scenario::S2);
    m_dtree->yes->no->no->yes->yes = std::make_shared<cf::TreeNode<DecisionData>>(Scenario::S1);
}

Scenario ScenarioDetector::decision_tree(const DecisionData& data) const
{
    return cf::traverse<DecisionData>(m_dtree, data);
}

void ScenarioDetector::init_knn()
{
    // site.width | site.height | number streets | s1-4.width    | number vehicle
    //     sw     |     sh      |      ns        | l | r | t | b |        ns 
    m_knn_data_set = {
        { cf::ScenarioSituation{ {/*sw*/ 2.0, /*sh*/ 2.0, /*ns*/ 4.0, /*s*/ /*l*/ 2.0, /*r*/ 2.0, /*t*/ 2.0, /*b*/ 2.0, /*nv*/ 4.0 }, Scenario::S2 } },
        { cf::ScenarioSituation{ {/*sw*/ 2.0, /*sh*/ 2.0, /*ns*/ 4.0, /*s*/ /*l*/ 2.0, /*r*/ 2.0, /*t*/ 2.0, /*b*/ 2.0, /*nv*/ 3.0 }, Scenario::S2 } },
        { cf::ScenarioSituation{ {/*sw*/ 2.0, /*sh*/ 2.0, /*ns*/ 4.0, /*s*/ /*l*/ 2.0, /*r*/ 2.0, /*t*/ 2.0, /*b*/ 2.0, /*nv*/ 2.0 }, Scenario::S2 } },
        { cf::ScenarioSituation{ {/*sw*/ 2.0, /*sh*/ 2.0, /*ns*/ 3.0, /*s*/ /*l*/ 2.0, /*r*/ 2.0, /*t*/ 0.0, /*b*/ 1.0, /*nv*/ 2.0 }, Scenario::S1 } },
        { cf::ScenarioSituation{ {/*sw*/ 2.0, /*sh*/ 2.0, /*ns*/ 4.0, /*s*/ /*l*/ 2.0, /*r*/ 2.0, /*t*/ 2.0, /*b*/ 2.0, /*nv*/ 3.0 }, Scenario::S1 } }
    };

}

Scenario ScenarioDetector::knn_classify(const DecisionData& data) const
{
    return cf::knn_classify(m_knn_data_set, data, 3);
}
