#include <functional>
#include <memory>

#include "multi_truck_scenario/msg/event_site_data.hpp"
#include "multi_truck_scenario/msg/street_data.hpp"

#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "scenario_type.hpp"
#include "classification.hpp"

namespace cf {

std::vector<float> extract_features(const DecisionData& situation)
{
    std::vector<float> features;
    const auto& site = situation.second;
    const auto& vehicles = situation.first;

    features.push_back(site.width);
    features.push_back(site.height);
    features.push_back(site.num_streets);

    for (const mts_msgs::StreetData& s : site.streets)
    {
        features.push_back(static_cast<float>(s.width));
    }

    features.push_back(vehicles.size());

    return features;
} 

Scenario knn_classify(const std::vector<ScenarioSituation>& data_set, const DecisionData& input, const int k) 
{
    const auto query_features = extract_features(input);

    std::vector<std::pair<float, Scenario>> distances;
    for (const auto& data : data_set)
    {
        float dist = distance_func(query_features, data.features);
        distances.push_back({dist, data.label});
    }

    std::sort(distances.begin(), distances.end());

    std::unordered_map<Scenario, int> class_count;
    for (int i = 0; i < k && i < (int)distances.size(); i++)
    {
        class_count[distances[i].second]++;
    }

    const auto max_class = std::max_element(class_count.begin(), class_count.end(),
        [](const std::pair<Scenario, int>& a, const std::pair<Scenario, int>& b) {
            return a.second < b.second;
    });

    return max_class->first;
}

double distance_func(const std::vector<float>& a, const std::vector<float>& b)
{
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
    {
        sum += std::pow(a[i] - b[i], 2);
    }
    return std::sqrt(sum);
}


void BayesianNetwork::add_cpd(const CPD& cpd)
{
    m_cpds.insert({cpd.variable, cpd});
}

int BayesianNetwork::state_index(const std::vector<std::string>& states, const std::string& value)
{
    for (size_t i = 0; i < states.size(); ++i)
    {
        if (states[i] == value) return i;
    }
    return -1;
}

double BayesianNetwork::query(std::string query_var, std::string query_val, std::map<std::string, std::string> evidence)
{
    double numerator = 0.0;
    double denominator = 0.0;

    for (const std::string& val : m_cpds[query_var].states)
    {
        std::map<std::string, std::string> e = evidence;
        e.insert({query_var, val});
        double p = full_joint_prob(e);
        RCLCPP_INFO(rclcpp::get_logger("tester"), "p: %f",p);

        if (val == query_val) numerator += p;
        denominator += p;
    }
    return numerator / denominator;
}

double BayesianNetwork::full_joint_prob(std::map<std::string, std::string> evidence)
{
    double prob = 1.0;

    for (const auto& [var, cpd] : m_cpds)
    {
        int row = state_index(cpd.states, evidence[var]);

        if (cpd.parents.empty())
        {
            prob *= cpd.probabilities[row][0];
        }
        else
        {
            int col = 0;
            int multiplier = 1;
            for (int i = (int)cpd.parents.size() - 1; i >= 0; --i)
            {
                std::string parent = cpd.parents[i];
                int idx = state_index(m_cpds[parent].states, evidence[parent]);
                col += idx * multiplier;
                multiplier *= cpd.parent_cardinalities[i];
            }
            prob *= cpd.probabilities[row][col];
        }
    }
    return prob;
}

}
