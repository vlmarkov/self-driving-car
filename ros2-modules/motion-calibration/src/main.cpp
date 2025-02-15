#include <motion-calibration/motion_calibration.h>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionCalibration>());
    rclcpp::shutdown();
    return 0;
}
