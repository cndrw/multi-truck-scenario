#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      m_grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_data", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto grid = nav_msgs::msg::OccupancyGrid();
        grid.info.height = 4;
        grid.info.width = 4;
        grid.info.resolution = 1;

        grid.header.stamp = rclcpp::Clock().now();
        grid.header.frame_id = "map_frame";

        grid.info.origin.position.x = 0;
        grid.info.origin.position.y = 0;
        grid.info.origin.position.z = 0;
        grid.info.origin.orientation.x = 0;
        grid.info.origin.orientation.y = 0;
        grid.info.origin.orientation.z = 0;
        grid.info.origin.orientation.w = 1;

        grid.data = {
            100, 0, 0, 100,
            -100, 0, 0, 0,
            0, 0, 0, 0,
            100, -100, 0, 100,
        };

        m_grid_pub->publish(grid);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_grid_pub;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}