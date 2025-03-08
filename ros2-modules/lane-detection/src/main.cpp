#include <lane-detection/lane_detection.h>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto cfg = PubSubCfg{
        .name = LaneDetection::kName,
        .topic_publiser = std::string(LaneDetection::kName) + "Out",
        .topic_subscription = std::string(LaneDetection::kName) + "In",
        .duration = 500ms
    };
    auto pub_sub_node = std::make_shared<BasePubSubNode>(cfg);
    auto manual_control = std::make_shared<LaneDetection>(pub_sub_node);

    rclcpp::spin(pub_sub_node);
    rclcpp::shutdown();

    return 0;
}
