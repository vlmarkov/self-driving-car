#include <main-pipeline/ros_module.h>

#include <manual-control/ros_module.h>
#include <motion-calibration/ros_module.h>

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace {

constexpr auto DEFAULT_TICK_TIMEOUT = 1000ms;

} // namespace

MainPipeline::MainPipeline()
    : Node(MainPipeline::kName)
{
    auto cb = [this]() -> void { this->tick_(); };
    timer_ = this->create_wall_timer(DEFAULT_TICK_TIMEOUT, cb);

    manual_control_module_ = add_module_(ManualControl::kName);
    motion_calibration_module_ = add_module_(MotionCalibration::kName);
}

void MainPipeline::add_module(std::string name) {
    if (module_name_set_.contains(name))
        throw std::runtime_error("can't add already existed module!");

    auto new_module = add_module_(name);
    module_name_set_.insert(name);
    module_subscriptions_.push_back(new_module);
}

void MainPipeline::tick_() {
    if (manual_control_module_->data.is_auto_pilot_on == false) {
        transfer_message_("OFF", manual_control_module_, motion_calibration_module_);
        return;
    }

    for (size_t i = 0; i < module_subscriptions_.size(); ++i) {
        auto& from = module_subscriptions_[i];
        auto& to = (i == module_subscriptions_.size() - 1) ? motion_calibration_module_ : module_subscriptions_[i + 1];

        transfer_message_("ON", from, to);
    }
}

void MainPipeline::transfer_message_(std::string_view auto_pilot, std::shared_ptr<SubscribedModule> from, std::shared_ptr<SubscribedModule> to) {
    RCLCPP_INFO(this->get_logger(), "[AUTO-PILOT %s] from `%s` to `%s` acceleration %f steer %f",
        auto_pilot.data(),
        from->name.c_str(),
        to->name.c_str(),
        from->data.acceleration,
        from->data.steering
    );

    to->publisher->publish(from->data);
}

std::shared_ptr<SubscribedModule> MainPipeline::add_module_(std::string name) {
    auto module = std::make_shared<SubscribedModule>();

    auto cb = [module](interfaces::msg::MotionVector::UniquePtr mv) -> void {
        module->data = *mv.get();
    };

    module->name = name;
    module->publisher = this->create_publisher<interfaces::msg::MotionVector>(name + "In" , DEFAULT_QUEUE_SIZE);
    module->subscription = this->create_subscription<interfaces::msg::MotionVector>(name + "Out", DEFAULT_QUEUE_SIZE, cb);

    return module;
}
