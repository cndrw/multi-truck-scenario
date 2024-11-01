#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class Map : public rclcpp::Node
{
  public:
    Map() : rclcpp::Node("map")
    
    {
        m_grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_data", 10);
        m_timer = this->create_wall_timer(
          send_frequenzy, std::bind(&Map::timer_callback, this)
        );

        m_vehicle_sub = this->create_subscription<multi_truck_scenario::msg::VehicleBaseData>("vecicle_base_data", 10,
          std::bind(&Map::vehicle_position_callback, this, std::placeholders::_1)
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
            0, 0, 0, 0,
            0, 0, 0, 0,
            100, 0, 0, 100,
        };

      // update vehicle position on the grid 
      for (const auto& vehicle : m_vehicles)
      {
        const int x = vehicle.second->position.point.x;
        const int y = vehicle.second->position.point.y;
        grid.data[x * y] = -100;
      }


        m_grid_pub->publish(grid);
    }

    void vehicle_position_callback(const multi_truck_scenario::msg::VehicleBaseData::SharedPtr vehicle_data)
    {
      const auto key = vehicle_data->vin;
      if (m_vehicles.count(key) == 0) 
      {
        m_vehicles.emplace(key, vehicle_data);
      }
      else 
      {
        m_vehicles[key] = vehicle_data;
      }
    }

    std::chrono::milliseconds send_frequenzy = 500ms;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::unordered_map<int, multi_truck_scenario::msg::VehicleBaseData::SharedPtr> m_vehicles;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_grid_pub;
    rclcpp::Subscription<multi_truck_scenario::msg::VehicleBaseData>::SharedPtr m_vehicle_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Map>());
  rclcpp::shutdown();
  return 0;
}