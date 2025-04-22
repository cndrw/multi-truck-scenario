#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "multi_truck_scenario/msg/s2_solution.hpp"
#include "event_site.hpp"

using namespace std::chrono_literals;
namespace mts_msgs = multi_truck_scenario::msg;

enum Colors {
    Red = -128,      // rot für -128 bis -70
    Yellow = -70,    // gelb für -70 bis -2
    Grey = 0,        // grau für -1 bis 80
    Black = 100,     // schwarz für 80 bis 100
    Green = 127      // grün (101 bis 127) für Gewinnerfahrzeug in s2_solution_callback
};

class Map : public rclcpp::Node
{
  public:
    Map() : rclcpp::Node("map")
    {
      handle_parameters();
      m_resolution = 1;

      m_grid.resize(m_height * m_width);
      m_static_map.reserve(m_height * m_width);

      m_static_map = map_color_parameter(); // Get the static map

      m_grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_data", 10);
      m_cube_pub = this->create_publisher<visualization_msgs::msg::Marker>("cube_dta", 10);
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

    void handle_parameters(){
        this->declare_parameter("height", 0);
        this->declare_parameter("width", 0);

        m_height = this->get_parameter("height").as_int();
        m_width = this->get_parameter("width").as_int();

        // TODO: set event site data
        // m_event_sites.push_back() 
    }

    std::vector<Colors> map_color_parameter() {
        // Declare the parameter and fetch it
        this->declare_parameter<std::vector<int64_t>>("static_map", {}); // Declare as int64_t
        std::vector<int64_t> static_map_raw = this->get_parameter("static_map").as_integer_array();

        // Convert to Colors
        std::vector<Colors> static_map_colors;
        for (const auto &value : static_map_raw) {
            if (value >= -128 && value <= 127) {
                static_map_colors.push_back(static_cast<Colors>(value));
            } else {
                RCLCPP_WARN(this->get_logger(), "Value %ld is out of range for Colors enum. Skipping.", value);
            }
        }

        // Return the populated vector
        return static_map_colors;
        
    }

  private:
    void draw_car(float x, float y, int id)
    {
      auto cube = visualization_msgs::msg::Marker();
      cube.header.stamp = rclcpp::Clock().now();
      cube.header.frame_id = "map_frame";
      cube.action = visualization_msgs::msg::Marker::ADD;
      cube.type = visualization_msgs::msg::Marker::CUBE;
      cube.ns = std::to_string(id);
      cube.pose.position.x = 0.5 + x;
      cube.pose.position.y = 0.5 + y;
      cube.pose.position.z = 0.5;
      cube.color = m_car_visuals[id];
      cube.scale.x = 1.0;
      cube.scale.y = 1.0;
      cube.color.a = 1.0;
      cube.scale.z = 1.0;
      m_cube_pub->publish(cube);
    }
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
/**       if (is_out_of_bound(vehicle_data->position.point))
        {
            return;
        } */

        const auto key = vehicle_data->vin;
        if (m_vehicles.count(key) == 0) 
        {
            m_vehicles.emplace(key, vehicle_data);
            m_color_map.emplace(key, Colors::Red);
            auto c = std_msgs::msg::ColorRGBA();
            c.r = 1;
            m_car_visuals.emplace(key, c);
        }
        else 
        {
            m_vehicles[key] = vehicle_data;
        }

        auto pos = vehicle_data->position.point;
        draw_car(pos.x, pos.y, vehicle_data->vin);

        set_vehicle_color(key);
    }

    bool is_out_of_bound(const geometry_msgs::msg::Point& pos)
    {
        return pos.x >= m_width || pos.x < 0 || pos.y >= m_height || pos.y < 0;
    }

    void s2_solution_callback(const mts_msgs::S2Solution::SharedPtr solution)
    {
        const auto vin = solution->winner_vin;

        // set the winner green
        m_color_map[vin] = Colors::Green;
        auto c = m_car_visuals[vin];
        c.g = 1;
        c.r = 0;
        m_car_visuals[vin] = c;

        set_vehicle_color(vin);
    }

    void set_vehicle_color(const int vin)
    {
        const auto& pos = m_vehicles[vin]->position.point;

        // round to the second decimal place and then floor them to fit the grid
        const auto rpos_x = std::floor(std::ceil(pos.x * 100.0) / 100.0);
        const auto rpos_y = std::floor(std::ceil(pos.y * 100.0) / 100.0);

        m_grid[rpos_x + rpos_y * m_width] = static_cast<int8_t>(m_color_map[vin]); // convert Colors enum to int8_t
    }

    void add_to_map(const std::vector<Colors>& grid)
    {
        for (size_t i = 0; i < m_grid.size(); i++)
        {
            if (grid[i] == Colors::Grey) continue; // Grey means empty
            m_grid[i] = static_cast<int8_t>(grid[i]); // Map Colors enum to int8_t
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
    std::vector<Colors> m_static_map;
    std::vector<int8_t> m_grid;
    std::unordered_map<int, int> m_color_map;
    std::unordered_map<int, std_msgs::msg::ColorRGBA> m_car_visuals;
    std::vector<EventSite> m_event_sites;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_grid_pub;
    rclcpp::Subscription<mts_msgs::VehicleBaseData>::SharedPtr m_vehicle_sub;
    rclcpp::Subscription<mts_msgs::S2Solution>::SharedPtr m_s2_solution_sub;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_cube_pub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Map>());
  rclcpp::shutdown();
  return 0;
}
