#include <common/base_publish_node.h>

class LineDetection : public BasePublishNode
{
public:
    LineDetection();
    ~LineDetection() = default;
};

LineDetection::LineDetection()
    : BasePublishNode("LineDetection")
{
    // Nothing so far
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineDetection>());
    rclcpp::shutdown();
    return 0;
}
