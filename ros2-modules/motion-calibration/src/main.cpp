#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MotionCalibration : public rclcpp::Node
{
public:
    MotionCalibration();
    ~MotionCalibration() = default;

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

MotionCalibration::MotionCalibration() : Node("MotionCalibration")
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("MotionCalibration", 100);
        
    auto timer_callback = [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "MotionCalibration";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
    };

    timer_ = this->create_wall_timer(500ms, timer_callback);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionCalibration>());
    rclcpp::shutdown();
    return 0;
}
