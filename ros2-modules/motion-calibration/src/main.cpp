#include <motion-calibration/ros_module.h>

using namespace std::chrono_literals;

void run(std::stop_token stop_token, std::shared_ptr<MotionCalibration> mc) {
    while(!stop_token.stop_requested()) {
        mc->process_motion_vector();
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto cfg = PubSubCfg{
        .name = MotionCalibration::kName,
        .topic_publiser = std::string(MotionCalibration::kName) + "Out",
        .topic_subscription = std::string(MotionCalibration::kName) + "In",
        .duration = 500ms
    };
    auto pub_sub_node = std::make_shared<BasePubSubNode>(cfg);
    auto motion_calibration = std::make_shared<MotionCalibration>(pub_sub_node);

    std::jthread thread(run, motion_calibration);

    rclcpp::spin(pub_sub_node);
    rclcpp::shutdown();

    return 0;
}
