#include <lane-detection/lane_detection_module.h>

int main(int argc, char* argv[]) {
    auto frame = cv::imread(argv[1], cv::IMREAD_UNCHANGED);
    if (frame.empty()) {
        return -1;
    }

    LaneDetectionModule lm({});
    auto status = lm.detect_lane(std::move(frame));

    cv::imshow("Lane Detection Status", status.frame);
    std::cout << status.direction << std::endl;
    std::cout << status.steer_angle << std::endl;

    if (cv::waitKey(3000) >= 0) {
        return 0;
    }

    return 0;
}
