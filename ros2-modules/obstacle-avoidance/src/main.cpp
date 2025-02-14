#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ObstacleAvoidance : public rclcpp::Node
{
public:
    ObstacleAvoidance();
    ~ObstacleAvoidance() = default;

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

ObstacleAvoidance::ObstacleAvoidance() : Node("ObstacleAvoidance")
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("ObstacleAvoidance", 100);
        
    auto timer_callback = [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "ObstacleAvoidance";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
    };

    timer_ = this->create_wall_timer(500ms, timer_callback);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidance>());
    rclcpp::shutdown();
    return 0;
}
