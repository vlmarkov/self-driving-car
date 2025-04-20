#include <lane-detection/lane_detection_module.h>

#include <raspicam/raspicam_cv.h>

void setup_camera (int argc, char **argv, raspicam::RaspiCam_Cv &camera) {
    camera.set(cv::CAP_PROP_FRAME_WIDTH,  ("-w", argc, argv,   400));
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, ("-h", argc, argv,   240));
    camera.set(cv::CAP_PROP_BRIGHTNESS,   ("-br", argc, argv,  50));
    camera.set(cv::CAP_PROP_CONTRAST,     ("-co", argc, argv,  50));
    camera.set(cv::CAP_PROP_SATURATION,   ("-sa", argv, argc,  50));
    camera.set(cv::CAP_PROP_GAIN,         ("-g", argc, argv,   50));
    camera.set(cv::CAP_PROP_FPS,          ("-fps", argc, argv, 0));
}

void open_camera(raspicam::RaspiCam_Cv &camera) {
    if (!camera.open()) {
        std::cerr << "Error opening the camera" << std::endl;
        std::exit(-1);
    }
}

void stop_camera(raspicam::RaspiCam_Cv &camera) {
    std::cout << "Stop camera..." << std::endl;
    camera.release();
}

cv::Mat get_frame(raspicam::RaspiCam_Cv &camera) {
    cv::Mat frame;

    camera.grab();
    camera.retrieve(frame);

    return frame;
}

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
