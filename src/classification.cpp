#include <functional>
#include <memory>

#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "scenario_type.hpp"
#include "classification.hpp"

namespace cf {


Scenario knn_classify(const std::vector<ScenarioSituation>& data_set, const ScenarioSituation& input, const int k)
{
    std::vector<std::pair<double, Scenario>> distances; 
    for (const auto& p : data_set)
    {
        const auto dist = distance_func(p, input);
        distances.push_back({dist, p.scenario});
    }

    std::sort(distances.begin(), distances.end());

    // get most frequent class from k nearest neigbors
    std::map<Scenario, int> class_count;
    for (int i = 0; i < k; ++i)
    {
        class_count[distances[i].second]++;
    }

    const auto max_class = std::max_element(class_count.begin(), class_count.end(),
        [](const std::pair<Scenario, int>& a, const std::pair<Scenario, int>& b) {
            return a.second < b.second;
        });

    return max_class->first;
}

double distance_func(const ScenarioSituation& a, const ScenarioSituation& b)
{
    std::vector<double> features = {
        a.event_site.position.x,
        a.event_site.position.y,
        a.event_site.position.z,
        (double)a.event_site.height,
        (double)a.event_site.width,
        // a.event_site.streets,
        (double)a.vehicles.size()
    };


    double sum = 0.0;
    for (size_t i = 0; i < features.size(); ++i)
    {
        // sum += std::pow(a[i] - b[i], 2);
    }
    return std::sqrt(sum);
}

}
