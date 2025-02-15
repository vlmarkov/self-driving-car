#include <main-pipeline/main_pipeline.h>

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

MainPipeline::MainPipeline()
    : Node("MainPipeline")
{
    // Nothig so far
}

void MainPipeline::add_module(const std::string& name) {
    auto cb = [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    };

    module_subscriptions.push_back(this->create_subscription<std_msgs::msg::String>(name, 100, cb));
}
