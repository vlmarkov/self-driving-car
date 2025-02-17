#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/motion_vector.hpp"

class BasePublishNode : public rclcpp::Node
{
public:
    BasePublishNode(const std::string& topic_name);
    ~BasePublishNode() = default;

private:
    const std::string name_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
