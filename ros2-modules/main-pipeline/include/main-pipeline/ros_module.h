#pragma once

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/motion_vector.hpp"

#include <vector>
#include <unordered_map>

struct SubscribedModule {
    std::string name;
    interfaces::msg::MotionVector data;

    rclcpp::Publisher<interfaces::msg::MotionVector>::SharedPtr publisher;
    rclcpp::Subscription<interfaces::msg::MotionVector>::SharedPtr subscription;
};

class MainPipeline : public rclcpp::Node
{
public:
    static constexpr auto kName{"MainPipeline"};

    MainPipeline();
    ~MainPipeline() = default;

    void add_module(const std::string& name);

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<SubscribedModule> manual_control_module_;
    std::shared_ptr<SubscribedModule> motion_calibration_module_;

    std::unordered_set<std::string> module_name_set_;
    std::vector<std::shared_ptr<SubscribedModule>> module_subscriptions_;

    void tick_();
    void transfer_message_(const std::string& auto_pilot, std::shared_ptr<SubscribedModule> from, std::shared_ptr<SubscribedModule> to);
    std::shared_ptr<SubscribedModule> add_module_(const std::string& name);
};
