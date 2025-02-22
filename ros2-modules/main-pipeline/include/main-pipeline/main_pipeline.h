#pragma once

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/motion_vector.hpp"

#include <vector>

struct SubscribedModule {
    std::string name_;
    interfaces::msg::MotionVector from_prev_module_;
    interfaces::msg::MotionVector to_next_module_;

    rclcpp::Publisher<interfaces::msg::MotionVector>::SharedPtr publisher_;
    rclcpp::Subscription<interfaces::msg::MotionVector>::SharedPtr subscription_;
};

class MainPipeline : public rclcpp::Node
{
public:
    static constexpr auto kName{"MainPipeline"};

    MainPipeline();
    ~MainPipeline() = default;

    void add_module(const std::string& name);
    void transfer_message(const size_t index, const interfaces::msg::MotionVector& msg);

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::shared_ptr<SubscribedModule>> module_subscriptions_;

    void process_callback();
};
