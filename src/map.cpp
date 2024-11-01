#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


using namespace std::chrono_literals;

class Map : public rclcpp::Node
{
  public:
    Map() : rclcpp::Node("map")
    
    {
        m_grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_data", 10);
        timer_ = this->create_wall_timer(
          500ms, std::bind(&Map::timer_callback, this)
        );
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
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_grid_pub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Map>());
  rclcpp::shutdown();
  return 0;
}