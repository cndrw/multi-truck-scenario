#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "multi_truck_scenario/msg/s2_solution.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;
namespace mts_msgs = multi_truck_scenario::msg;


enum Colors {
    Red = -128,      // rot für -128 bis -70
    Yellow = -70,    // gelb für -70 bis -2
    Grey = 0,       // grau für -1 bis 80
    Black = 100,      // schwarz für 80 bis 100
    Green = 127      // grün (101 bis 127) für Gewinnerfahrzeug in s2_solution_callback
};

class Map : public rclcpp::Node
{
  public:
    Map() : rclcpp::Node("map")
    {
      m_width = 4;
      m_height = 4;
      m_resolution = 1;

      m_grid.resize(m_height * m_width);
      m_static_map.reserve(m_height * m_width);

      m_static_map = {
          Colors::Black, Colors::Grey, Colors::Grey, Colors::Black,
          Colors::Grey, Colors::Grey, Colors::Grey, Colors::Grey,
          Colors::Grey, Colors::Grey, Colors::Grey, Colors::Grey,
          Colors::Black, Colors::Grey, Colors::Grey, Colors::Black,
      };


      m_grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_data", 10);
      m_timer = this->create_wall_timer(
        send_frequenzy, std::bind(&Map::timer_callback, this)
      );

      m_vehicle_sub = this->create_subscription<mts_msgs::VehicleBaseData>("vehicle_base_data", 10,
        std::bind(&Map::vehicle_position_callback, this, std::placeholders::_1)
      );

      m_s2_solution_sub = this->create_subscription<mts_msgs::S2Solution>("s2_solution", 10,
        std::bind(&Map::s2_solution_callback, this, std::placeholders::_1)
      );
    }

  private:
    void timer_callback()
    {
      auto grid = nav_msgs::msg::OccupancyGrid();
      grid.info.height = m_height;
      grid.info.width = m_width;
      grid.info.resolution = m_resolution;

      grid.header.stamp = rclcpp::Clock().now();
      grid.header.frame_id = "map_frame";

      grid.info.origin.position.x = 0;
      grid.info.origin.position.y = 0;
      grid.info.origin.position.z = 0;
      grid.info.origin.orientation.x = 0;
      grid.info.origin.orientation.y = 0;
      grid.info.origin.orientation.z = 0;
      grid.info.origin.orientation.w = 1;

      add_to_map(m_static_map);
      grid.data = m_grid;

      m_grid_pub->publish(grid);

      clear_map();
    }

    void vehicle_position_callback(const mts_msgs::VehicleBaseData::SharedPtr vehicle_data)
    {
      const auto key = vehicle_data->vin;
      if (m_vehicles.count(key) == 0) 
      {
        m_vehicles.emplace(key, vehicle_data);
        m_color_map.emplace(key, Colors::Red);
      }
      else 
      {
        m_vehicles[key] = vehicle_data;
      }

      set_vehicle_color(key);
    }

    void s2_solution_callback(const mts_msgs::S2Solution::SharedPtr solution)
    {
        const auto vin = solution->winner_vin;

        // set the winner green
        m_color_map[vin] = Colors::Green;

        set_vehicle_color(vin);
    }

    void set_vehicle_color(const int vin)
    {
        const auto& pos = m_vehicles[vin]->position.point;
        m_grid[pos.x + pos.y * m_width] = m_color_map[vin];
    }

    void add_to_map(const std::vector<int8_t>& grid)
    {
      for (size_t i = 0; i < m_grid.size(); i++)
      {
        if (grid[i] == Colors::Grey) continue;
        m_grid[i] = grid[i];
      }
    }

    void clear_map()
    {
      for (auto& cell : m_grid)
      {
        cell = 0;
      }
    }

    int m_width;
    int m_height;
    int m_resolution;
    std::chrono::milliseconds send_frequenzy = 500ms;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::unordered_map<int, mts_msgs::VehicleBaseData::SharedPtr> m_vehicles;
    std::vector<int8_t> m_static_map;
    std::vector<int8_t> m_grid;
    std::unordered_map<int, int> m_color_map;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_grid_pub;
    rclcpp::Subscription<mts_msgs::VehicleBaseData>::SharedPtr m_vehicle_sub;
    rclcpp::Subscription<mts_msgs::S2Solution>::SharedPtr m_s2_solution_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Map>());
  rclcpp::shutdown();
  return 0;
}