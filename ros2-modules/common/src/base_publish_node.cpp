#include <common/base_publish_node.h>

#include <chrono>

using namespace std::chrono_literals;

BasePublishNode::BasePublishNode(const std::string& name)
    : Node(name)
    , name_(name)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>(name_, 100);
        
    auto timer_callback = [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = name_;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
    };

    timer_ = this->create_wall_timer(500ms, timer_callback);
}
