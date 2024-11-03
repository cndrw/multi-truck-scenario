#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "multi_truck_scenario/msg/vehicle_base_data.hpp"

using namespace std::chrono_literals;
namespace mts_msgs = multi_truck_scenario::msg;

enum Indicator { off, left, right, warning };

class Vehicle : public rclcpp::Node
{
  public:
    Vehicle() : rclcpp::Node("vehicle")
    {
        
        handle_parameters();
        m_vehicle_pub = this->create_publisher<mts_msgs::VehicleBaseData>("vehicle_base_data", 10);

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

        // base package informations
        double m_speed;
        double m_direction;
        int m_vin;
        geometry_msgs::msg::PointStamped m_position;
        Indicator m_indicator_state = Indicator::off;

        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Publisher<mts_msgs::VehicleBaseData>::SharedPtr m_vehicle_pub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vehicle>());
  rclcpp::shutdown();
  return 0;
}