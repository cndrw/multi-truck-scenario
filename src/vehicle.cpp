#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


using namespace std::chrono_literals;

class Vehicle : public rclcpp::Node
{
  public:
    Vehicle() : rclcpp::Node("vehicle")
    {
        
        handle_parameters();
        position_pub_ = this->create_publisher<geometry_msgs::msg::Point>("vehicle_position", 10);

        // Initialisiere Position
        m_position.x = 0.0;
        m_position.y = 0.0;
        m_position.z = 0.0;

        // Timer, der die Position alle 100 ms veröffentlicht
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Vehicle::publish_position, this)
        );
    }

    void handle_parameters()
    {
        this->declare_parameter("position_x", 0.0);
        this->declare_parameter("position_y", 0.0);
        this->declare_parameter("position_z", 0.0);
        this->declare_parameter("direction_x", 0.0);
        this->declare_parameter("direction_y", 0.0);
        this->declare_parameter("direction_z", 0.0);
        this->declare_parameter("vin", 0);
        this->declare_parameter("speed", 0.0);
        this->declare_parameter("indicator_state", 0);

        auto pos_point = geometry_msgs::msg::Point();
        pos_point.x = this->get_parameter("position_x").as_double();
        pos_point.x = this->get_parameter("position_y").as_double();
        pos_point.x = this->get_parameter("position_z").as_double();

        auto dir_point = geometry_msgs::msg::Point();
        dir_point.x = this->get_parameter("direction_x").as_double();
        dir_point.x = this->get_parameter("direction_y").as_double();
        dir_point.x = this->get_parameter("direction_z").as_double();

        int vin = this->get_parameter("vin").as_int();
        double speed = this->get_parameter("speed").as_double();
        int indicator_state = this->get_parameter("indicator_state").as_int();
    }

    void set_position(double x, double y, double z)
    {
        m_position.x = x;
        m_position.y = y;
        m_position.z = z;
        RCLCPP_INFO(this->get_logger(), "Aktuelle Position: (%.2f, %.2f, %.2f)", x, y, z);
    }
    
    private:
        void publish_position()
        {
            // Veröffentlichen der aktuellen Position
            RCLCPP_INFO(this->get_logger(), "Aktuelle Position: (%.2f, %.2f, %.2f)", m_position.x, m_position.y, m_position.z);
            position_pub_->publish(m_position);
        }

        geometry_msgs::msg::Point m_position;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vehicle>());
  rclcpp::shutdown();
  return 0;
}