#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <vector>

struct SubscribedModule {
    std::string name_;
    std::string from_prev_module_;
    std::string to_next_module_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

class MainPipeline : public rclcpp::Node
{
public:
    MainPipeline();
    ~MainPipeline() = default;

    void add_module(const std::string& name);
    void transfer_message(const size_t index, const std::string& msg);

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::shared_ptr<SubscribedModule>> module_subscriptions_;

    void process_callback();
};
