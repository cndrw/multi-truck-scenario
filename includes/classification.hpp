#pragma once

#include <functional>
#include <memory>

#include "multi_truck_scenario/msg/vehicle_base_data.hpp"
#include "scenario_type.hpp"
#include "event_site.hpp"

namespace cf { // classification

namespace mts_msgs = multi_truck_scenario::msg;

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

struct ScenarioSituation
{
    Scenario scenario;
    EventSite event_site;
    std::vector<mts_msgs::VehicleBaseData> vehicles;
};

Scenario knn_classify(const std::vector<ScenarioSituation>& data_set, const ScenarioSituation& input, const int k);

double distance_func(const ScenarioSituation& a, const ScenarioSituation& b);

}
