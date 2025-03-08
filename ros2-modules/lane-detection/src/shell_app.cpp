#include <lane-detection/lane_detection_module.h>

int main(int argc, char* argv[]) {
    auto frame = cv::imread(argv[1], cv::IMREAD_UNCHANGED);
    if (frame.empty()) {
        return -1;
    }

    LaneDetectionModule lm({});
    auto lane_detection_info = lm.detectLane(std::move(frame));

    cv::imshow("Lane Detection", lane_detection_info.frame);
    std::cout << lane_detection_info.direction << std::endl;
    std::cout << lane_detection_info.steer_angle << std::endl;

    if (cv::waitKey(3000) >= 0) {
        return 0;
    }

    return 0;
}
