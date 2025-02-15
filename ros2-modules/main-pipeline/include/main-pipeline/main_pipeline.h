#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <vector>

class MainPipeline : public rclcpp::Node
{
public:
    MainPipeline();
    ~MainPipeline() = default;

    void add_module(const std::string& name);

private:
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> module_subscriptions;
};
