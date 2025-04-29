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
static constexpr auto DEG2RAD { M_PI / 180 };


class Vehicle : public rclcpp::Node
{
public:
    Vehicle(): rclcpp::Node("vehicle")
    {
        handle_parameters();
        m_scenario_solver.set_owner(m_vin);
        m_scenario_detector.set_implemenation(1, 0);
        m_scenario_detector.set_owner(m_vin);

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
            500ms, std::bind(&Vehicle::update, this)
        );
    }
    ~Vehicle() {}

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
        
        this->declare_parameter("scenario_detector", 0);
        this->declare_parameter("decision_algorithm", 0);
        
        m_position.point.x = this->get_parameter("position_x").as_double();
        m_position.point.y = this->get_parameter("position_y").as_double();
        m_position.point.z = this->get_parameter("position_z").as_double();

        m_position.point.z = this->get_parameter("position_z").as_double();

        m_direction = this->get_parameter("direction").as_double();

        m_vin = this->get_parameter("vin").as_int();
        m_speed = this->get_parameter("speed").as_double();
        m_indicator_state = (Indicator)this->get_parameter("indicator_state").as_int();
        m_engine_state = (Engine)this->get_parameter("engine_state").as_int();

        // RCLCPP_INFO(get_logger(), "detector: %d", this->get_parameter("scenario_detector").as_int());
    }

    void update()
    {
        if (!m_is_active)
        {
            return;
        }

        std::vector<mts_msgs::VehicleBaseData> tmp;
        tmp.reserve(m_nearby_vehicles.size());
        for (auto& v : m_nearby_vehicles)
        {
            tmp.push_back(*v.second);
        }

        if (tmp.size() != 0)
        {
            Scenario scenario = m_scenario_detector.check(tmp);
            const auto solution = m_scenario_solver.solve(scenario, tmp);

            if (solution == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "solution not valid");
                return;
            }

            auto solution_msg = mts_msgs::S2Solution();
            solution_msg.author_vin = m_vin;
            solution_msg.winner_vin = solution->winner_vin;
            m_solution_pub->publish(solution_msg);
        }


        move_vehicle();

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

    void move_vehicle()
    {
        static rclcpp::Time last_time = this->get_clock()->now();
        const auto now_time = this->get_clock()->now();
        const auto delta_time = now_time - last_time;

        // get driving direction
        const double dy = std::sin(m_direction * DEG2RAD);
        const double dx = std::cos(m_direction * DEG2RAD);

        const double delta_move = m_speed * delta_time.seconds();
        m_position.point.x += delta_move * dx;
        m_position.point.y += delta_move * dy;

        // RCLCPP_INFO(get_logger(), "pos.x: %f, pos.y: %f,  dx: %f, move: %f", m_position.point.x, m_position.point.y, dx, dx * delta_move);

        last_time = now_time;
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

        // "S2 filter" until scenario detector is build if you own vehicle is moving -> not calculating any sceario
        if (vehicle_data->speed != 0 || m_speed != 0)
        {
            return;
        }

        if (m_solution_delay - this->get_clock()->now().seconds() > 0)
        {
            return;
        }

        // Handle the received message
        const auto key = vehicle_data->vin;
        if (m_nearby_vehicles.count(key) == 0) 
        {
            m_nearby_vehicles.emplace(key, vehicle_data);
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

        if (m_nearby_vehicles.size() == 4)
        {
            m_nearby_vehicles.clear();
            m_solution_vins.clear();
            m_count--;
            m_solution_delay = this->get_clock()->now().seconds() + m_delay_time;

            if (solution->winner_vin == m_vin)
            {
                RCLCPP_INFO(this->get_logger(), "kill %d", m_vin);
                // m_is_active = false;
                m_speed = 1.0;
            }
           return; 
        }

        if (m_solution_vins.size() < m_nearby_vehicles.size())
        {
            return;
        }

        // check if all vins are the same
        bool all_equal = std::all_of(m_solution_vins.begin(), m_solution_vins.end(), [this](int vin) {
            return vin == this->m_solution_vins[0];
        });

        if (all_equal)
        {
            m_nearby_vehicles.clear();
            m_solution_vins.clear();
            m_count--;
            m_solution_delay = this->get_clock()->now().seconds() + m_delay_time;

            if (solution->winner_vin == m_vin)
            {
                RCLCPP_INFO(this->get_logger(), "kill %d", m_vin);
                // m_is_active = false;
                m_speed = 1.0;
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
        std::unordered_map<int, mts_msgs::VehicleBaseData::SharedPtr> m_nearby_vehicles;
        rclcpp::Publisher<mts_msgs::VehicleBaseData>::SharedPtr m_vehicle_pub;
        rclcpp::Subscription<mts_msgs::VehicleBaseData>::SharedPtr m_vehicle_sub;

        rclcpp::Publisher<mts_msgs::S2Solution>::SharedPtr m_solution_pub;
        rclcpp::Subscription<mts_msgs::S2Solution>::SharedPtr m_solution_sub;
        std::vector<int> m_solution_vins;

        ScenarioSolver m_scenario_solver;
        ScenarioDetector m_scenario_detector;

        // temp
        size_t m_count = 2;
        double m_solution_delay;
        double m_delay_time = 2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vehicle>());
  rclcpp::shutdown();
  return 0;
}
