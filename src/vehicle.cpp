#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "multi_truck_scenario/msg/s2_solution.hpp"

using namespace std::chrono_literals;
namespace mts_msgs = multi_truck_scenario::msg;

enum Indicator { off, left, right, warning };

class Vehicle : public rclcpp::Node
{
public:
    Vehicle() : rclcpp::Node("vehicle")
    {
        handle_parameters();
        // Publisher der die Daten der Instanz veröffentlicht
        m_vehicle_pub = this->create_publisher<mts_msgs::VehicleBaseData>("vehicle_base_data", 10);

        m_solution_pub = this->create_publisher<mts_msgs::S2Solution>("s2_solution", 10);

        // Subscriber der die Werte der anderen Fahrzeuge empfängt
        m_vehicle_sub = this->create_subscription<mts_msgs::VehicleBaseData>(
            "vehicle_base_data", 10, std::bind(&Vehicle::vehicle_position_callback, this, std::placeholders::_1)
        );

        // Timer, der die Position alle 100 ms veröffentlicht
        m_timer = this->create_wall_timer(
            100ms, std::bind(&Vehicle::publish_vehicle, this)
        );
    }

    void handle_parameters()
    {
        this->declare_parameter("position_x", 0.0);
        this->declare_parameter("position_y", 0.0);
        this->declare_parameter("position_z", 0.0);
        this->declare_parameter("direction", 0.0);
        this->declare_parameter("vin", 0);
        this->declare_parameter("speed", 0.0);
        this->declare_parameter("indicator_state", 0);
        
        m_position.point.x = this->get_parameter("position_x").as_double();
        m_position.point.y = this->get_parameter("position_y").as_double();
        m_position.point.z = this->get_parameter("position_z").as_double();

        m_direction = this->get_parameter("direction").as_double();

        m_vin = this->get_parameter("vin").as_int();
        m_speed = this->get_parameter("speed").as_double();
        m_indicator_state = (Indicator)this->get_parameter("indicator_state").as_int();
    }

    void set_position(geometry_msgs::msg::PointStamped point)
    {
        m_position = point;
        m_position.header.stamp = rclcpp::Clock().now();
    }

    void set_speed(double speed)
    {
        m_speed = speed;
    }

    void set_direction(double direction)
    {
        m_direction = direction;
    }

    void set_vin(int vin)
    {
        m_vin = vin;
    }
    
private:
    void publish_vehicle()
    {
        // Veröffentlichen der aktuellen Position
        // RCLCPP_INFO(this->get_logger(), "Aktuelle Position: (%.2f, %.2f, %.2f)", m_position.x, m_position.y, m_position.z);
        m_position.header.stamp = rclcpp::Clock().now();

        // build the base data package 
        auto vehicle_base_data = mts_msgs::VehicleBaseData();
        vehicle_base_data.position = m_position;
        vehicle_base_data.direction = m_direction;
        vehicle_base_data.speed = m_speed;
        vehicle_base_data.vin = m_vin;
        vehicle_base_data.indicator_state = m_indicator_state;

        m_vehicle_pub->publish(vehicle_base_data);
    }

    void solve_scenario_s2()
    {
        int winner_vin = -1;

        for (const auto& v1 : m_vehicles)
        {
            double angle = v1.second->direction; // 0, 90, 180, 270
            int count = 0;

            for (const auto& v2 : m_vehicles)
            {
                if (v1 == v2) continue;

                double wanted_angle = angle == 270.0 ? 0.0 : angle + 90.0;

                // is there a car to the right? Then v1 can't be the winner 
                if (wanted_angle == v2.second->direction) break;

                count++;
            }

            // if all cars are not "right" than this one has to drive
            if (static_cast<size_t>(count) == m_vehicles.size() - 1)
            {
                winner_vin = v1.second->vin;
            }
        }

        auto solution = mts_msgs::S2Solution();
        solution.header.stamp = rclcpp::Clock().now();
        solution.author_vin = m_vin;
        solution.winner_vin = winner_vin;
        
        m_solution_pub->publish(solution);
    }


    void vehicle_position_callback(const mts_msgs::VehicleBaseData::SharedPtr vehicle_data)
    {
            /* this method shall do:
                - read position & direction & indicator state & vin of up to 2 other vehicles
            */
            // Handle the received message
            // RCLCPP_INFO(this->get_logger(), "Vehicle Info: \n\t VIN: %d \n\t Position: \n\t\t X: %.2f \n\t\t Y: %.2f \n\t\t Z: %.2f \n\t Direction: %.2f \n\t Speed: %.2f \n\t Indicator State: %d", msg->vin, msg->position.point.x, msg->position.point.y, msg->position.point.z, msg->direction, msg->speed, msg->indicator_state);
            const auto key = vehicle_data->vin;
            if (m_vehicles.count(key) == 0) 
            {
                m_vehicles.emplace(key, vehicle_data);
                std::cout << (m_vehicles[key]->vin) << std::endl;
                if (m_vehicles.size() == 3)
                {
                    solve_scenario_s2();
                }
            }
    }
        // base package informations
        double m_speed;
        double m_direction;
        int m_vin;
        geometry_msgs::msg::PointStamped m_position;
        Indicator m_indicator_state = Indicator::off;

        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Publisher<mts_msgs::VehicleBaseData>::SharedPtr m_vehicle_pub;
        rclcpp::Subscription<mts_msgs::VehicleBaseData>::SharedPtr m_vehicle_sub;
        std::unordered_map<int, mts_msgs::VehicleBaseData::SharedPtr> m_vehicles;
        rclcpp::Publisher<mts_msgs::S2Solution>::SharedPtr m_solution_pub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vehicle>());
  rclcpp::shutdown();
  return 0;
}