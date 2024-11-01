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
    Vehicle() : rclcpp::Node("Vehicle")

    {
        position_pub_ = this->create_publisher<geometry_msgs::msg::Point>("vehicle_position", 10);

        // Initialisiere Position
        position_.x = 0.0;
        position_.y = 0.0;
        position_.z = 0.0;

        // Timer, der die Position alle 100 ms veröffentlicht
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Vehicle::publish_position, this)
        );
    }

    void set_position(double x, double y, double z)
    {
        position_.x = x;
        position_.y = y;
        position_.z = z;
        RCLCPP_INFO(this->get_logger(), "Aktuelle Position: (%.2f, %.2f, %.2f)", x, y, z);
    }
    
    private:
        void publish_position()
        {
            // Veröffentlichen der aktuellen Position
            RCLCPP_INFO(this->get_logger(), "Aktuelle Position: (%.2f, %.2f, %.2f)", position_.x, position_.y, position_.z);
            position_pub_->publish(position_);
        }

        geometry_msgs::msg::Point position_;
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