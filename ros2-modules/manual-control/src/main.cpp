#include <common/base_publish_node.h>

class ManualControl : public BasePublishNode
{
public:
    ManualControl();
    ~ManualControl() = default;
};

ManualControl::ManualControl()
    : BasePublishNode("ManualControl")
{
    // Nothing so far
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualControl>());
    rclcpp::shutdown();
    return 0;
}
