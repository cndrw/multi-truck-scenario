#pragma once

#include <functional>
#include <memory>

#include "multi_truck_scenario/msg/event_site_data.hpp"

#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "scenario_type.hpp"
#include "event_site.hpp"

namespace cf { // classification

namespace mts_msgs = multi_truck_scenario::msg;

using DecisionData = std::pair<std::vector<mts_msgs::VehicleBaseData>, mts_msgs::EventSiteData>;

struct ScenarioSituation
{
    // es = event_site | v = vehicles | s = street
    // alignment:
    // [es.width, es.height, num_streets, s1.width, s2.width, s3.width, s4.width, v.size()] 
    std::vector<float> features;
    Scenario label;
};

template <typename T>
struct TreeNode
{
    std::function<bool(const T&)> decision;
    std::shared_ptr<TreeNode> yes;
    std::shared_ptr<TreeNode> no;
    Scenario result;
    bool is_leaf = true;

    TreeNode(const Scenario res) : result(res), is_leaf(true) { }
    TreeNode(std::function<bool(const T&)> cond) : decision(cond), is_leaf(false) { }
};

template <typename T>
Scenario traverse(const std::shared_ptr<TreeNode<T>>& node, const T& data)
{
    if (node->is_leaf)
    {
        return node->result;
    }

    if (node->decision(data))
    {
        return traverse(node->yes, data);
    }
    else
    {
        return traverse(node->no, data);
    }
}

Scenario knn_classify(const std::vector<ScenarioSituation>& data_set, const DecisionData& input, const int k);
double distance_func(const std::vector<float>& a, const std::vector<float>& b);

}
