#include <main-pipeline/main_pipeline.h>

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

MainPipeline::MainPipeline()
    : Node("MainPipeline")
{
    auto cb = [this]() -> void { this->process_callback(); };
    timer_ = this->create_wall_timer(1000ms, cb);
}

void MainPipeline::add_module(const std::string& name) {
    const auto module_index = module_subscriptions_.size();
    auto new_module = std::make_shared<SubscribedModule>();
    new_module->name_ = name;

    auto cb = [this, new_module, module_index](std_msgs::msg::String::UniquePtr msg) -> void {
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

        new_module->to_next_module_ = msg->data.c_str();

        auto message = std_msgs::msg::String();
        message.data = new_module->from_prev_module_;
        new_module->publisher_->publish(message);
        this->transfer_message(module_index, new_module->to_next_module_);
    };

    new_module->publisher_ = this->create_publisher<std_msgs::msg::String>(name + "from_MainPipeline" , 100);
    new_module->subscription_ = this->create_subscription<std_msgs::msg::String>(name, 100, cb);

    module_subscriptions_.push_back(new_module);
}

void MainPipeline::process_callback() {
    for (const auto& s: module_subscriptions_) {
        RCLCPP_INFO(this->get_logger(), "[%s]'%s'/'%s'", s->name_.c_str(), s->from_prev_module_.c_str(), s->to_next_module_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), " ");
}

void MainPipeline::transfer_message(const size_t index, const std::string& msg) {
    if (module_subscriptions_.empty())
        return;

    if (index == module_subscriptions_.size() - 1)
        return;

    module_subscriptions_[index + 1]->from_prev_module_ = msg;
}
