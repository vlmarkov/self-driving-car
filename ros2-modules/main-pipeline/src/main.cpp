#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

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

MainPipeline::MainPipeline() : Node("MainPipeline")
{
    auto topic_callback = [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    };

    manual_control_ = this->create_subscription<std_msgs::msg::String>("ManualControl", 100, topic_callback);
    path_planning_ = this->create_subscription<std_msgs::msg::String>("PathPlanning", 100, topic_callback);
    obstacle_avoidance_ = this->create_subscription<std_msgs::msg::String>("ObstacleAvoidance", 100, topic_callback);
    line_detection_ = this->create_subscription<std_msgs::msg::String>("LineDetection", 100, topic_callback);
    motion_calibration_ = this->create_subscription<std_msgs::msg::String>("MotionCalibration", 100, topic_callback);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainPipeline>());
    rclcpp::shutdown();
    return 0;
}
