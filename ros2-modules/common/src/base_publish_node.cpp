#include <common/base_publish_node.h>

#include <chrono>

using namespace std::chrono_literals;

BasePublishNode::BasePublishNode(const std::string& topic_name)
    : Node(topic_name)
    , name_(topic_name)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>(name_, 100);
        
    auto timer_callback = [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = name_;
        RCLCPP_INFO(this->get_logger(), "I Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
    };

    timer_ = this->create_wall_timer(500ms, timer_callback);

    auto cb = [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    };

    subscription_ = this->create_subscription<std_msgs::msg::String>(name_ + "from_MainPipeline", 100, cb);
}
