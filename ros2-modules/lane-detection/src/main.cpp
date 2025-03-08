#include <lane-detection/lane_detection.h>

#include <thread>

using namespace std::chrono_literals;

void run(std::stop_token stop_token, std::shared_ptr<LaneDetection> ld) {
    // TODO: add here actual work with camera

    while(!stop_token.stop_requested()) {
        auto frame = cv::imread("left_turn.jpg", cv::IMREAD_UNCHANGED); // TODO: this is only for test purpose
        ld->process_frame(std::move(frame));

        if (cv::waitKey(30) >= 0) // TODO: work with delay
            break;
    }
}

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
    auto lane_detection = std::make_shared<LaneDetection>(pub_sub_node);

    std::jthread thread(run, lane_detection);

    rclcpp::spin(pub_sub_node);
    rclcpp::shutdown();

    return 0;
}
