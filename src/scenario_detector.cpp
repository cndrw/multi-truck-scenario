#include "scenario_detector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "multi_truck_scenario/srv/get_event_site_distance.hpp"
#include <iostream>

using namespace std::chrono_literals;
namespace mts_msgs = multi_truck_scenario::msg;
namespace mts_srvs = multi_truck_scenario::srv;

void ScenarioDetector::set_implemenation(const int impl)
{
    m_implementation = impl;
}

ScenarioDetector::ScenarioDetector()
{
    impl[0] = std::bind(&ScenarioDetector::check_1, this);
    impl[1] = std::bind(&ScenarioDetector::check_2, this);
}

Scenario ScenarioDetector::check()
{
    return impl[m_implementation]();
}

Scenario ScenarioDetector::check_1()
{
    return Scenario();
}

Scenario ScenarioDetector::check_2()
{
    auto vehicle = mts_msgs::VehicleBaseData();
    vehicle.speed = 5;
    vehicle.position.point.x = 0;
    vehicle.position.point.y = 0;

    // service von map -> get_closest_(ereignisstelle)
    // calculate distance from vehicle to (ereignisstelle)

    FValue velocity = velocity_fuzzy_func(vehicle.speed); 
    FValue distance = distance_fuzzy_func(get_distance_event_site(vehicle));

    return Scenario();
}

FValue ScenarioDetector::velocity_fuzzy_func(float velocity)
{
    return
    {
        .first = velocity_standing(velocity),
        .second = velocity_slow(velocity),
        .third = velocity_fast(velocity)
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
    if (velocity > 2 && velocity < 7)
    {
        return 1.0 / 5.0 * (velocity - 2.0);
    }

    if (velocity > 2 && velocity <= 7)
    {
        return 1.0;
    }

    if (velocity > 10 && velocity < 20)
    {
        return - 1.0 / 10.0 * (velocity - 20.0);
    }

    if (velocity <= 2 || velocity >= 20)
    {
        return 0.0;
    }
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

    if (velocity >= 30)
    {
        return 1.0;
    }
}

FValue ScenarioDetector::distance_fuzzy_func(const float distance)
{
    return
    {
        .first = distance_inside(distance),
        .second = distance_close(distance),
        .third = distance_far(distance)
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

    if (distance >= 0)
    {
        return 0.0;
    }
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

    if (distance <= -1 || distance >= 10)
    {
        return 0.0;
    }
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

    if (distance <= 5)
    {
        return 0.0;
    }
}

float ScenarioDetector::get_distance_event_site(const mts_msgs::VehicleBaseData& vehicle)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("map_requester");
    rclcpp::Client<mts_srvs::GetEventSiteDistance>::SharedPtr client =
        node->create_client<mts_srvs::GetEventSiteDistance>("get_event_site_distance");

    auto request = std::make_shared<mts_srvs::GetEventSiteDistance::Request>();
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "res: %f", result.get()->distance);
        return result.get()->distance;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
        return 0;
    }
}
