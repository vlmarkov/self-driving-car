#include <main-pipeline/main_pipeline.h>

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

MainPipeline::MainPipeline()
    : Node(MainPipeline::kName)
{
    auto cb = [this]() -> void { this->process_callback(); };
    timer_ = this->create_wall_timer(1000ms, cb);
}

void MainPipeline::add_module(const std::string& name) {
    const auto module_index = module_subscriptions_.size();
    auto new_module = std::make_shared<SubscribedModule>();
    new_module->name_ = name;

    auto cb = [this, new_module, module_index](interfaces::msg::MotionVector::UniquePtr mv) -> void {
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

        new_module->to_next_module_ = *mv.get();

        auto prev_mv = interfaces::msg::MotionVector();
        prev_mv = new_module->from_prev_module_;
        new_module->publisher_->publish(prev_mv);

        this->transfer_message(module_index, new_module->to_next_module_);
    };

    new_module->publisher_ = this->create_publisher<interfaces::msg::MotionVector>(name + "In" , 100);
    new_module->subscription_ = this->create_subscription<interfaces::msg::MotionVector>(name + "Out", 100, cb);

    module_subscriptions_.push_back(new_module);
}

void MainPipeline::process_callback() {
    for (const auto& s: module_subscriptions_) {
        RCLCPP_INFO(this->get_logger(),
            "[%s] %d %f:%f/%d %f:%f", s->name_.c_str(),
            s->from_prev_module_.is_auto_pilot_on,
            s->from_prev_module_.acceleration,
            s->from_prev_module_.steering,
            s->to_next_module_.is_auto_pilot_on,
            s->to_next_module_.acceleration,
            s->to_next_module_.steering
        );
    }
    RCLCPP_INFO(this->get_logger(), " ");
}

void MainPipeline::transfer_message(const size_t index, const interfaces::msg::MotionVector& mv) {
    if (module_subscriptions_.empty())
        return;

    if (index == module_subscriptions_.size() - 1)
        return;

    module_subscriptions_[index + 1]->from_prev_module_ = mv;
}
