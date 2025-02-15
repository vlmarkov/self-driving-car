#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MainPipeline : public rclcpp::Node
{
public:
    MainPipeline();
    ~MainPipeline() = default;

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manual_control_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_planning_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obstacle_avoidance_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr line_detection_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_calibration_;
};
