#include <common/base_publish_node.h>

class ObstacleAvoidance : public BasePublishNode
{
public:
    ObstacleAvoidance();
    ~ObstacleAvoidance() = default;
};

ObstacleAvoidance::ObstacleAvoidance()
    : BasePublishNode("ObstacleAvoidance")
{
    // Nothing so far
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidance>());
    rclcpp::shutdown();
    return 0;
}
