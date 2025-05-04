#pragma once

#include <functional>
#include <memory>
#include "scenario_type.hpp"

namespace cf { // classification

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

}
