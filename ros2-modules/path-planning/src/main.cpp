#include <common/base_publish_node.h>

class PathPlanning : public BasePublishNode
{
public:
    PathPlanning();
    ~PathPlanning() = default;
};

PathPlanning::PathPlanning()
    : BasePublishNode("PathPlanning")
{
    // Nothing so far
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanning>());
    rclcpp::shutdown();
    return 0;
}
