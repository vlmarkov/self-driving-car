#include <main-pipeline/main_pipeline.h>

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

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