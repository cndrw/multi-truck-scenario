#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "multi_truck_scenario/srv/get_event_site_distance.hpp"
#include "multi_truck_scenario/srv/get_event_site_id.hpp"
#include "multi_truck_scenario/msg/solution.hpp"
#include "multi_truck_scenario/msg/street_data.hpp"

#include "tutils.h"
#include "event_site.hpp"

using namespace std::chrono_literals;
namespace mts_msgs = multi_truck_scenario::msg;
namespace mts_srvs = multi_truck_scenario::srv;

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

        m_grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_data", 10);
        m_cube_pub = this->create_publisher<visualization_msgs::msg::Marker>("cube_dta", 10);
        m_arrow_pub = this->create_publisher<visualization_msgs::msg::Marker>("arrow_dta", 10);
        m_border_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("cube_dta_arr", 10);

        m_timer = this->create_wall_timer(
            send_frequenzy, std::bind(&Map::timer_callback, this)
        );

        
        m_solution_sub = this->create_subscription<mts_msgs::Solution>("solution", 10,
            std::bind(&Map::solution_callback, this, std::placeholders::_1)
        );

        using namespace std::placeholders;
        m_vehicle_sub = this->create_subscription<mts_msgs::VehicleBaseData>("vehicle_base_data", 10,
            std::bind(&Map::vehicle_position_callback, this, _1)
        );

        m_esite_dist_srv = this->create_service<mts_srvs::GetEventSiteDistance>("get_event_site_distance",
            std::bind(&Map::get_event_site_distance, this, _1, _2));

        m_esite_id_srv = this->create_service<mts_srvs::GetEventSiteID>("get_event_site_id",
            std::bind(&Map::get_event_site_id, this, _1, _2));
        
        m_static_map = map_color_parameter(); // Get the static map
    }

    void handle_parameters()
    {
        this->declare_parameter("height", 0);
        this->declare_parameter("width", 0);

        m_height = this->get_parameter("height").as_int();
        m_width = this->get_parameter("width").as_int();

        this->declare_parameter<std::vector<int64_t>>("crossing_width_values", {});
        this->declare_parameter<std::vector<int64_t>>("crossing_height_values", {});
        this->declare_parameter<std::vector<int64_t>>("crossing_bot_left_x_values", {});
        this->declare_parameter<std::vector<int64_t>>("crossing_bot_left_y_values", {});
        this->declare_parameter<std::vector<int64_t>>("street_width_left", {});
        this->declare_parameter<std::vector<int64_t>>("street_width_right", {});
        this->declare_parameter<std::vector<int64_t>>("street_width_top", {});
        this->declare_parameter<std::vector<int64_t>>("street_width_bottom", {});

        const auto width_values = this->get_parameter("crossing_width_values").as_integer_array();
        const auto height_values = this->get_parameter("crossing_height_values").as_integer_array();
        const auto bot_left_x_values = this->get_parameter("crossing_bot_left_x_values").as_integer_array();
        const auto bot_left_y_values = this->get_parameter("crossing_bot_left_y_values").as_integer_array();
        const auto street_width_left = this->get_parameter("street_width_left").as_integer_array();
        const auto street_width_right = this->get_parameter("street_width_right").as_integer_array();
        const auto street_width_top = this->get_parameter("street_width_top").as_integer_array();
        const auto street_width_bottom = this->get_parameter("street_width_bottom").as_integer_array();

        // Set content of struct Street
        size_t num_streets = street_width_left.size();
        if (street_width_right.size() != num_streets ||
            street_width_top.size() != num_streets ||
            street_width_bottom.size() != num_streets) 
        {
            RCLCPP_ERROR(this->get_logger(), "Mismatch in size of street parameter arrays!");
            return;
        }

        // Set content of struct EventSite
        size_t num_sites = width_values.size();
        if (height_values.size() != num_sites || 
        bot_left_x_values.size() != num_sites || 
        bot_left_y_values.size() != num_sites) 
        {
            RCLCPP_ERROR(this->get_logger(), "Mismatch in size of crossing parameter arrays!");
            return;
        }

        for (size_t i = 0; i < num_sites; ++i) {
            EventSite site;
            site.width = width_values[i];
            site.height = height_values[i];
            site.position.x = bot_left_x_values[i];
            site.position.y = bot_left_y_values[i];
            site.position.z = 0.0; // Assuming z is always 0 for the event site
            site.streets = { // Assuming 4 streets for each event site
              Street{static_cast<int>(street_width_left[i])},
              Street{static_cast<int>(street_width_right[i])},
              Street{static_cast<int>(street_width_top[i])},
              Street{static_cast<int>(street_width_bottom[i])}
            };
            m_event_sites.emplace(i, site);
            RCLCPP_INFO(this->get_logger(), 
                "EventSite %zu: pos=(%.1f, %.1f), size=(%d x %d), Streets [L=%d, R=%d, T=%d, B=%d]", 
                i,
               site.position.x,
               site.position.y,
               site.width,
               site.height,
               site.streets[0].width,
               site.streets[1].width,
               site.streets[2].width,
               site.streets[3].width
              );

        }
    }

    std::vector<Colors> map_color_parameter()
    {
        // Declare the parameter and fetch it
        this->declare_parameter<std::vector<int64_t>>("static_map", {}); // Declare as int64_t
        std::vector<int64_t> static_map_raw = this->get_parameter("static_map").as_integer_array();

        // Convert to Colors
        std::vector<Colors> static_map_colors;
        for (const auto &value : static_map_raw)
        {
            if (value >= -128 && value <= 127)
            {
                static_map_colors.push_back(static_cast<Colors>(value));

            } else {
                RCLCPP_WARN(this->get_logger(), "Value %ld is out of range for Colors enum. Skipping.", value);
            }
        }

        // Return the populated vector
        return static_map_colors;
    }

  private:

    void draw_border()
    {
      int x = 0;
      int y = 0;
      std::vector<visualization_msgs::msg::Marker> blocks;
    
      
      for (size_t i = 0; i < m_static_map.size(); i++)
      {
        if (i % m_width == 0 && i != 0)
        {
          x = 0;
          y++;
        }

        if (m_static_map[i] == Colors::Black)
        {
          blocks.push_back(draw_border_block(x, y, 0.25, i));
        }

        x++;
      }

      auto border = visualization_msgs::msg::MarkerArray();
      border.markers = blocks;
  
      m_border_pub->publish(border);
    }

    visualization_msgs::msg::Marker draw_border_block(const float x, const float y, const float height, const int id)
    {
      auto cube = visualization_msgs::msg::Marker();
      cube.header.stamp = this->now();
      cube.header.frame_id = "map_frame";
      cube.action = visualization_msgs::msg::Marker::ADD;
      cube.type = visualization_msgs::msg::Marker::CUBE;
      cube.ns = "border_";
      cube.id = id;
      cube.pose.position.x = 0.5 + x;
      cube.pose.position.y = 0.5 + y;
      cube.pose.position.z = height / 2.0;
      cube.color.r = 0.0;
      cube.color.g = 0.0;
      cube.color.b = 0.0;
      cube.color.a = 1.0;
      cube.scale.x = 1.0;
      cube.scale.y = 1.0;
      cube.scale.z = height;
      return cube;
    }

    void draw_car(float x, float y, int id)
    {
      auto cube = visualization_msgs::msg::Marker();
      cube.header.stamp = this->now();
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
      cube.lifetime = rclcpp::Duration(2, 0);
      m_cube_pub->publish(cube);
    }

    void draw_heading_arrow(const geometry_msgs::msg::Point& pos, float direction_angle, int id)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map_frame";
        marker.header.stamp = this->now();
        marker.ns = "arrows";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        auto end = geometry_msgs::msg::Point();
        end.x = std::cos(direction_angle * tutils::DEG2RAD);
        end.y = std::sin(direction_angle * tutils::DEG2RAD);
        end.z = 0.0;

        auto mid_point = tutils::add(pos, 0.5);

        marker.points.push_back(mid_point);
        marker.points.push_back(tutils::add(mid_point, end));

        marker.scale.x = 0.1;  // Schaftdurchmesser
        marker.scale.y = 0.2;   // Pfeilkopfdurchmesser
        marker.scale.z = 0.2;   // Pfeilkopflänge

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration(2, 0);
        m_arrow_pub->publish(marker);
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
        draw_border();

        grid.data = m_grid;


        m_grid_pub->publish(grid);

        clear_map();
    }

    void vehicle_position_callback(const mts_msgs::VehicleBaseData::SharedPtr vehicle_data)
    {
        if (is_out_of_bound(vehicle_data->position.point))
        {
          return;
        }

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
        draw_heading_arrow(vehicle_data->position.point, vehicle_data->direction, vehicle_data->vin);
    }

    bool is_out_of_bound(const geometry_msgs::msg::Point& pos)
    {
        return pos.x >= m_width || pos.x < 0 || pos.y >= m_height || pos.y < 0;
    }

    void solution_callback(const mts_msgs::Solution::SharedPtr solution)
    {
        const auto vin = solution->winner_vin;

        // set the winner green
        m_color_map[vin] = Colors::Green;
        auto c = m_car_visuals[vin];
        c.g = 1;
        c.r = 0;
        m_car_visuals[vin] = c;
    }

    void get_event_site_id(const mts_srvs::GetEventSiteID::Request::SharedPtr request,
                                      mts_srvs::GetEventSiteID::Response::SharedPtr response)
    {
        const auto& pos = request->position;
        std::vector<std::pair<int, EventSite>> sites(m_event_sites.begin(), m_event_sites.end());

        std::sort(sites.begin(), sites.end(), [this, pos](const auto& s1, const auto& s2) {
            return this->calc_distance(s1.second, pos) < this->calc_distance(s2.second, pos);
        });

        response->id = sites[0].first;

        const auto& site = sites[0].second;
        response->event_site.position = site.position;

        int street_count = std::count_if(site.streets.begin(), site.streets.end(), [](const auto& s) {
            return s.width != 0;
        });
        response->event_site.num_streets = street_count;

        for (const auto& street : site.streets)
        {
            auto street_data = mts_msgs::StreetData();
            street_data.width = street.width;
            response->event_site.streets.push_back(street_data);
        }
    }

    void get_event_site_distance(const mts_srvs::GetEventSiteDistance::Request::SharedPtr request,
                                      mts_srvs::GetEventSiteDistance::Response::SharedPtr response)
    {
        const auto& event_site = m_event_sites[request->event_site_id];
        response->distance = calc_distance(event_site, request->position);
    }

    // calculate the distance from an position to the given event side with an box sdf
    double calc_distance(const EventSite& event_site, const geometry_msgs::msg::PointStamped& pos)
    {
        const auto& box_pos = event_site.position;
        float cx = box_pos.x + event_site.width * 0.5f;
        float cy = box_pos.y + event_site.height * 0.5f;

        // const auto dx = std::max(pos.point.x - box_pos.x, double { 0 });
        const auto dx = std::fabs(pos.point.x - cx) - event_site.width * 0.5f;
        // const auto dy = std::max(pos.point.y - box_pos.y, double { 0 });
        const auto dy = std::fabs(pos.point.y - cy) - event_site.height * 0.5f;

        float outside = std::sqrt(std::max(dx, 0.0) * std::max(dx, 0.0) +
                                  std::max(dy, 0.0) * std::max(dy, 0.0));
        const auto inside = std::min(std::max(dx, dy), 0.0);
        return outside + inside;
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

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_grid_pub;
    rclcpp::Subscription<mts_msgs::VehicleBaseData>::SharedPtr m_vehicle_sub;
    rclcpp::Subscription<mts_msgs::Solution>::SharedPtr m_solution_sub;

    std::unordered_map<int, EventSite> m_event_sites;
    rclcpp::Service<mts_srvs::GetEventSiteDistance>::SharedPtr m_esite_dist_srv;
    rclcpp::Service<mts_srvs::GetEventSiteID>::SharedPtr m_esite_id_srv;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_cube_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_arrow_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_border_pub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Map>());
  rclcpp::shutdown();
  return 0;
}
