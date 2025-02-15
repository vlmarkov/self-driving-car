#include <common/base_publish_node.h>

class MotionCalibration : public BasePublishNode
{
public:
    MotionCalibration();
    ~MotionCalibration() = default;
};

MotionCalibration::MotionCalibration()
    : BasePublishNode("MotionCalibration")
{
    // Nothing so far
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionCalibration>());
    rclcpp::shutdown();
    return 0;
}
