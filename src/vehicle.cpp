#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath> // für std::sqrt
#include <random>

#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "multi_truck_scenario/msg/s2_solution.hpp"

#include "scenario_solver.hpp"
#include "scenario_detector.hpp"

using namespace std::chrono_literals;
namespace mts_msgs = multi_truck_scenario::msg;

enum class Indicator { off, left, right, warning };
enum class Engine { on, off };

static constexpr auto RAD2DEG { 180 / M_PI };


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

         m_solution_sub = this->create_subscription<mts_msgs::S2Solution>(
            "s2_solution", 10, std::bind(&Vehicle::s2_solution_callback, this, std::placeholders::_1)
        );

        m_timer = this->create_wall_timer(
            500ms, std::bind(&Vehicle::publish_vehicle, this)
        );
    }

    bool is_active()
    {
        return m_is_active;
    }
    
private:
    void handle_parameters()
    {
        this->declare_parameter("position_x", 0.0);
        this->declare_parameter("position_y", 0.0);
        this->declare_parameter("position_z", 0.0);
        this->declare_parameter("direction", 0.0);
        this->declare_parameter("vin", 0);
        this->declare_parameter("speed", 0.0);
        this->declare_parameter("indicator_state", 0);
        this->declare_parameter("engine_state", 0);
        
        m_position.point.x = this->get_parameter("position_x").as_double();
        m_position.point.y = this->get_parameter("position_y").as_double();
        m_position.point.z = this->get_parameter("position_z").as_double();

        m_direction = this->get_parameter("direction").as_double();

        m_vin = this->get_parameter("vin").as_int();
        m_speed = this->get_parameter("speed").as_double();
        m_indicator_state = (Indicator)this->get_parameter("indicator_state").as_int();
        m_engine_state = (Engine)this->get_parameter("engine_state").as_int();
    }

    void publish_vehicle()
    {
        if (!m_is_active)
        {
            return;
        }

        // Veröffentlichen der aktuellen Position
        // RCLCPP_INFO(this->get_logger(), "Aktuelle Position: (%.2f, %.2f, %.2f)", m_position.x, m_position.y, m_position.z);
        m_position.header.stamp = rclcpp::Clock().now();

        // build the base data package 
        auto vehicle_base_data = mts_msgs::VehicleBaseData();
        vehicle_base_data.header.stamp = rclcpp::Clock().now();
        vehicle_base_data.engine_state = static_cast<int>(m_engine_state);
        vehicle_base_data.position = m_position;
        vehicle_base_data.direction = m_direction;
        vehicle_base_data.speed = m_speed;
        vehicle_base_data.vin = m_vin;
        vehicle_base_data.indicator_state = static_cast<int>(m_indicator_state);

        m_vehicle_pub->publish(vehicle_base_data);
    }

   geometry_msgs::msg::PointStamped substract(geometry_msgs::msg::PointStamped& p1, geometry_msgs::msg::PointStamped& p2)
   {
        auto tmp = geometry_msgs::msg::PointStamped();
        tmp.point.x = p2.point.x - p1.point.x;
        tmp.point.y = p2.point.y - p1.point.y;
        return tmp;
   }

    void pick_random_vehicle()
    {
        bool is_smallest_vin = std::all_of(m_vehicles.begin(), m_vehicles.end(), [this](const auto v) {
           return m_vin <= v.second->vin;  
        });

        if (is_smallest_vin)
        {
            std::srand(std::time(0)); 
            int rnd_vin = (std::rand() % 4) + 1; 


            
            auto solution = mts_msgs::S2Solution();
            solution.header.stamp = rclcpp::Clock().now();
            solution.author_vin = m_vin;
            solution.winner_vin = rnd_vin;
            
            m_solution_pub->publish(solution);
        }
    }

    void solve_scenario_s2()
    {
        int winner_vin = -1;
        for (const auto& v1 : m_vehicles)
        {
            size_t count = 0;
            // get the adjustment value so that v1.direction - adjustment_value equals 90
            const auto adjust_angle = 90 - v1.second->direction;
            const auto adjusted_v1_angle= v1.second->direction + adjust_angle; 

            for (const auto& v2 : m_vehicles)
            {
                if (v1 == v2) continue;

                const auto diff = substract(v1.second->position, v2.second->position);
                
                auto diff_angle = std::atan2(diff.point.y, diff.point.x) * RAD2DEG;
                diff_angle += adjust_angle; // also adjust the angle of the differenz vector

                if (diff_angle < 0)
                {
                    diff_angle += 360;
                }

                if (diff_angle >= adjusted_v1_angle)
                {
                    count++;
                }
            }

             // if all cars are not "right" than this one has to drive
            if (count == m_vehicles.size() - 1)
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
    if(!m_is_active)
        {
            return;
        }

    auto current_time = this->get_clock()->now();
    
    auto time_diff = current_time - vehicle_data->header.stamp;
    
    // difference over 200 ms -> ignoring the message
    if (time_diff > 200ms)
    {
        return;  // ignoring
    }

    // filter to consider only active vehicles
    if (vehicle_standard_filter(vehicle_data) == false)
    {
        return;
    }

    // Handle the received message
    const auto key = vehicle_data->vin;
    if (m_vehicles.count(key) == 0) 
    {
        m_vehicles.emplace(key, vehicle_data);
        std::cout << (m_vehicles[key]->vin) << std::endl;
        if (m_vehicles.size() == m_count)
                {
                    if (m_count == 4)
                    {
                        pick_random_vehicle();
                    }
                    else
                    {
                        solve_scenario_s2();
                    }
                }
    }
}

    bool vehicle_standard_filter(const mts_msgs::VehicleBaseData::SharedPtr vehicle_data)
    {
        if (vehicle_data->engine_state == static_cast<signed char>(Engine::off))
        {
            return false;
        }

        double dx = vehicle_data->position.point.x - m_position.point.x;
        double dy = vehicle_data->position.point.y - m_position.point.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance >= 1000.0)
        {
            return false;
        }

	return true;
    }

    void s2_solution_callback(const mts_msgs::S2Solution::SharedPtr solution)
    {
        if(!m_is_active)
        {
            return;
        }

        m_solution_vins.push_back(solution->winner_vin);

        if (m_vehicles.size() == 4)
        {
            m_vehicles.clear();
            m_solution_vins.clear();
            m_count--;

            if (solution->winner_vin == m_vin)
            {
                RCLCPP_INFO(this->get_logger(), "kill %d", m_vin);
                m_is_active = false;
            }
           return; 
        }

        if (m_solution_vins.size() < m_vehicles.size())
        {
            return;
        }

        // check if all vins are the same
        bool all_equal = std::all_of(m_solution_vins.begin(), m_solution_vins.end(), [this](int vin) {
            return vin == this->m_solution_vins[0];
        });

        if (all_equal)
        {
            m_vehicles.clear();
            m_solution_vins.clear();
            m_count--;

            if (solution->winner_vin == m_vin)
            {
                RCLCPP_INFO(this->get_logger(), "kill %d", m_vin);
                m_is_active = false;
            }
        }
    }

        bool m_is_active = true;
        // base package informations
        double m_speed;
        double m_direction;
        int m_vin;
        geometry_msgs::msg::PointStamped m_position;
        Indicator m_indicator_state = Indicator::off;
        Engine m_engine_state = Engine::on;

        rclcpp::TimerBase::SharedPtr m_timer;
        std::unordered_map<int, mts_msgs::VehicleBaseData::SharedPtr> m_vehicles;

        rclcpp::Publisher<mts_msgs::VehicleBaseData>::SharedPtr m_vehicle_pub;
        rclcpp::Subscription<mts_msgs::VehicleBaseData>::SharedPtr m_vehicle_sub;

        rclcpp::Publisher<mts_msgs::S2Solution>::SharedPtr m_solution_pub;
        rclcpp::Subscription<mts_msgs::S2Solution>::SharedPtr m_solution_sub;
        std::vector<int> m_solution_vins;

        // temp
        size_t m_count = 4;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vehicle>());
  rclcpp::shutdown();
  return 0;
}
