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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "multi_truck_scenario/msg/solution.hpp"
#include "multi_truck_scenario/msg/detection_proposal.hpp"

#include "truck_msgs/srv/zf_set_control_limits.hpp"
#include "truck_msgs/msg/zf_truck_init.hpp"

#include "scenario_solver.hpp"
#include "scenario_detector.hpp"
#include "tutils.h"

using namespace std::chrono_literals;
namespace mts_msgs = multi_truck_scenario::msg;

enum class Engine { on, off };

class Vehicle : public rclcpp::Node
{
public:
    Vehicle()
        : rclcpp::Node("vehicle"), m_startup_time(std::chrono::milliseconds(700))
    {
        handle_parameters();
        m_scenario_solver.set_owner(m_vin);
        m_scenario_detector.set_owner(m_vin);

        // Publisher der die Daten der Instanz veröffentlicht
        m_vehicle_pub = this->create_publisher<mts_msgs::VehicleBaseData>("vehicle_base_data", 10);

        m_solution_pub = this->create_publisher<mts_msgs::Solution>("solution", 10);

        // Subscriber der die Werte der anderen Fahrzeuge empfängt
        m_vehicle_sub = this->create_subscription<mts_msgs::VehicleBaseData>(
            "vehicle_base_data", 10, std::bind(&Vehicle::vehicle_position_callback, this, std::placeholders::_1)
        );

         m_solution_sub = this->create_subscription<mts_msgs::Solution>(
            "solution", 10, std::bind(&Vehicle::solution_callback, this, std::placeholders::_1)
        );

        m_vehicle_move_update = this->create_wall_timer(
            m_vehicle_move_period, std::bind(&Vehicle::move_vehicle, this)
        );

        m_vehicle_msg_update = this->create_wall_timer(
            m_msg_send_period, std::bind(&Vehicle::send_vehicle_msg, this)
        );

        m_dproposal_pub = this->create_publisher<mts_msgs::DetectionProposal>("scenario_proposal", 10);

        m_dproposal_sub = this->create_subscription<mts_msgs::DetectionProposal>(
            "scenario_proposal", 10, std::bind(&Vehicle::scenario_proposal_callback, this, std::placeholders::_1)
        );

        m_start_time = this->get_clock()->now();
        m_vehicle_update = this->create_wall_timer(
            m_system_update_period, std::bind(&Vehicle::update, this)
        );

        m_truck_init_pub = this->create_publisher<truck_msgs::msg::ZfTruckInit>("truck_init", 10);

        m_odometry_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "odometry_encoder_diff", 10, std::bind(&Vehicle::truck_odometry_callback, this, std::placeholders::_1)
        );


        // init zf truck 
        auto init_msg = truck_msgs::msg::ZfTruckInit();
        init_msg.psi = m_direction;
        init_msg.x = m_position.point.x;
        init_msg.y = m_position.point.y;
        m_truck_init_pub->publish(init_msg);
    }

    ~Vehicle() {}

    bool is_active()
    {
        return m_is_active;
    }
    
private:
    void handle_parameters()
    {
        this->declare_parameter("is_simulated", true);

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

        m_is_simulated = this->get_parameter("is_simulated").as_bool();
        
        m_position.point.x = this->get_parameter("position_x").as_double();
        m_position.point.y = this->get_parameter("position_y").as_double();
        m_position.point.z = this->get_parameter("position_z").as_double();

        m_position.point.z = this->get_parameter("position_z").as_double();

        m_direction = this->get_parameter("direction").as_double();

        m_vin = this->get_parameter("vin").as_int();
        m_speed = this->get_parameter("speed").as_double();
        m_indicator_state = (Indicator)this->get_parameter("indicator_state").as_int();
        RCLCPP_INFO(get_logger(), "init indicator: %d", m_indicator_state);
        m_engine_state = (Engine)this->get_parameter("engine_state").as_int();

        m_scenario_detector.set_implemenation(
            this->get_parameter("scenario_detector").as_int(),
            this->get_parameter("decision_algorithm").as_int() 
        );

    }

    void update()
    {
        const auto now = this->get_clock()->now();
        if ((now - m_start_time) < m_startup_time)
        {
            return;
        }

        if (!m_is_active)
        {
            return;
        }

        if (m_solution_delay - this->now().seconds() > 0)
        {
            return;
        }

        std::vector<mts_msgs::VehicleBaseData> tmp;
        tmp.reserve(m_nearby_vehicles.size());
        for (auto& v : m_nearby_vehicles)
        {
            tmp.push_back(*v.second);
        }

        // check scenario
        const auto res = m_scenario_detector.check(tmp);
        if (res.first == Scenario::None)
        {
            if (!m_driving_permission) RCLCPP_INFO(get_logger(), "No scenario found - grant driving permission");
            set_driving_permission(true);
            m_nearby_vehicles.clear();
            return;
        }

        for (const auto& v : res.second)
        {
            m_proposal_vehicles.emplace(v.vin, v);
        }

        m_detected_scenario = res.first;
        send_scenario_proposal(m_detected_scenario);

        m_nearby_vehicles.clear();
    }

    void send_vehicle_msg()
    {
        m_position.header.stamp = this->get_clock()->now();

        // build the base data package 
        auto vehicle_base_data = mts_msgs::VehicleBaseData();
        vehicle_base_data.header.stamp = this->get_clock()->now();
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

        if (m_driving_permission && m_is_simulated)
        {
            // get driving direction
            const double dy = std::sin(m_direction * tutils::DEG2RAD);
            const double dx = std::cos(m_direction * tutils::DEG2RAD);

            const double delta_move = m_speed * delta_time.seconds();
            m_position.point.x += delta_move * dx;
            m_position.point.y += delta_move * dy;
        }

        last_time = now_time;
    }

    void set_driving_permission(const bool value)
    {
        m_driving_permission = value;
        // /odometry/encoder
        m_speed = static_cast<double>(value);

        if (m_is_simulated)
        {
            return;
        }

        const auto client = create_client<truck_msgs::srv::ZfSetControlLimits>("set_control_limits");

        auto request = std::make_shared<truck_msgs::srv::ZfSetControlLimits::Request>();
        request->speed_max = (int)value * 50;

        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "service not available, waiting again...");
        }

        auto result = client->async_send_request(request);

        // Wait for the result.
        auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);

        if (status == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "set control succesfull");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
        }
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

        // "S2 filter" until scenario detector is build if you own vehicle is moving -> not calculating any scenario
        if (vehicle_data->speed != 0 || m_speed != 0)
        {
            return;
        }

        if (m_solution_delay - this->now().seconds() > 0)
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

    void truck_odometry_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr odometry)
    {
        m_position.point = odometry->pose.pose.position;
        // theoretisch m_direction = odometry->pose.pose.orientation
        // aber orientation ist in quaternionen und m_direction in grad
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

    void send_scenario_proposal(const Scenario scenario)
    {
        auto proposal = mts_msgs::DetectionProposal();
        proposal.owner_vin = m_vin;
        proposal.scenario = (int8_t)scenario;
        m_dproposal_pub->publish(proposal);
    }

    void scenario_proposal_callback(const mts_msgs::DetectionProposal::SharedPtr proposal)
    {    
        if (m_proposal_vehicles.count(proposal->owner_vin) == 0)
        {
            return;
        }

        m_proposal_buffer.push_back(*proposal);

        if (m_proposal_buffer.size() == m_proposal_vehicles.size())
        {
            bool all_equal = std::all_of(m_proposal_buffer.begin(), m_proposal_buffer.end(), [this](const mts_msgs::DetectionProposal& p) {
                return p.scenario == (int)this->m_detected_scenario;
            });

            if(all_equal)
            {
                solve_scenario();
            }
            else RCLCPP_INFO(get_logger(), "Not all scenario proposals are equal", m_vin);

            m_detected_scenario = Scenario::None;
            m_proposal_buffer.clear();
            // m_proposal_vehicles.clear();
        }
    }

    void solve_scenario()
    {        
        std::vector<mts_msgs::VehicleBaseData> tmp;
        tmp.reserve(m_proposal_vehicles.size());
        for (auto& v : m_proposal_vehicles)
        {
            tmp.push_back(v.second);
        }

        const auto solution = m_scenario_solver.solve(m_detected_scenario, tmp);
        if (solution == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "(%d) solution not valid", m_vin);
            return;
        }

        auto solution_msg = mts_msgs::Solution();
        solution_msg.author_vin = m_vin;
        solution_msg.winner_vin = solution->winner_vin;
        m_solution_pub->publish(solution_msg);
    }

    void solution_callback(const mts_msgs::Solution::SharedPtr solution)
    {
        if(!m_is_active)
        {
            return;
        }

        if (m_proposal_vehicles.count(solution->author_vin) == 0)
        {
            return;
        }

        m_solution_buffer.push_back(*solution);

        if (m_solution_buffer.size() == m_proposal_vehicles.size())
        {
            m_solution_delay = this->get_clock()->now().seconds() + m_delay_time;

            if (const auto res = compare_solutions(m_solution_buffer))
            {
                if (res->winner_vin == m_vin)
                {
                    RCLCPP_INFO(this->get_logger(), "Vehicle %d obtained driving permission", m_vin);
                    // grant driving permission 
                    set_driving_permission(true);
                }
                else 
                {
                    set_driving_permission(false);
                }
            }

            m_nearby_vehicles.clear();
            m_solution_buffer.clear();
            m_proposal_vehicles.clear();
        }
    }

    std::unique_ptr<mts_msgs::Solution> compare_solutions(const std::vector<mts_msgs::Solution>& solutions)
    {
        const auto it = std::find_if(solutions.begin(), solutions.end(), [this](const mts_msgs::Solution& s) {
            return s.winner_vin != VinFlags::Ignored;
        });

        // every vin is "ignored" -> no solution possible
        if (it == solutions.end())
        {
            return nullptr;
        }

        const auto& reference = *it;

        // check if all winner vins are the same
        bool all_equal = std::all_of(solutions.begin(), solutions.end(), [=](const mts_msgs::Solution& s) {
            return s.winner_vin == reference.winner_vin || s.winner_vin == VinFlags::Ignored;
        });

        return all_equal ? std::make_unique<mts_msgs::Solution>(reference) : nullptr;
    }

    bool m_is_active = true;
    bool m_driving_permission = false;
    bool m_is_simulated = true;

    // base package informations
    double m_speed;
    double m_direction;
    int m_vin;
    geometry_msgs::msg::PointStamped m_position;
    Indicator m_indicator_state = Indicator::Off;
    Engine m_engine_state = Engine::on;

    rclcpp::TimerBase::SharedPtr m_vehicle_update;
    rclcpp::TimerBase::SharedPtr m_vehicle_msg_update;
    rclcpp::TimerBase::SharedPtr m_vehicle_move_update;
    const std::chrono::seconds m_system_update_period = 1s;
    const std::chrono::milliseconds m_msg_send_period = 500ms;
    const std::chrono::milliseconds m_vehicle_move_period = 300ms;

    std::unordered_map<int, mts_msgs::VehicleBaseData::SharedPtr> m_nearby_vehicles;
    rclcpp::Publisher<mts_msgs::VehicleBaseData>::SharedPtr m_vehicle_pub;
    rclcpp::Subscription<mts_msgs::VehicleBaseData>::SharedPtr m_vehicle_sub;

    Scenario m_detected_scenario;
    std::unordered_map<int, mts_msgs::VehicleBaseData> m_proposal_vehicles;
    std::vector<mts_msgs::DetectionProposal> m_proposal_buffer;
    rclcpp::Publisher<mts_msgs::DetectionProposal>::SharedPtr m_dproposal_pub;
    rclcpp::Subscription<mts_msgs::DetectionProposal>::SharedPtr m_dproposal_sub;

    std::vector<mts_msgs::Solution> m_solution_buffer;
    rclcpp::Publisher<mts_msgs::Solution>::SharedPtr m_solution_pub;
    rclcpp::Subscription<mts_msgs::Solution>::SharedPtr m_solution_sub;
    std::vector<int> m_solution_vins;

    rclcpp::Publisher<truck_msgs::msg::ZfTruckInit>::SharedPtr m_truck_init_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_odometry_sub;

    ScenarioSolver m_scenario_solver;
    ScenarioDetector m_scenario_detector;

    rclcpp::Time m_start_time;
    rclcpp::Duration m_startup_time;

    // temp
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
