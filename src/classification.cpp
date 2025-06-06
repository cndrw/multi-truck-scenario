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

}
