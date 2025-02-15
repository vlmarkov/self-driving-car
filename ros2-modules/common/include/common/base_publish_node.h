#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class BasePublishNode : public rclcpp::Node
{
public:
    BasePublishNode(const std::string& name);
    ~BasePublishNode() = default;

private:
    const std::string name_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
