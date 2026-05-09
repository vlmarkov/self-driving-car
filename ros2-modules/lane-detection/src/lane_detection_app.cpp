#include <lane-detection/lane_detection_module.h>

#ifdef ENABLE_RASPBERRY_BUILD

#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>

void test_raspberry_camera() {
    uint32_t num_cams = LibcameraApp::GetNumberCameras();
    std::cout << "Found " << num_cams << " cameras." << std::endl;

    std::cout << "Sample program for LCCV video capture" << std::endl;
    std::cout << "Press ESC to stop." << std::endl;

    cv::Mat image = cv::Mat(0, 0, CV_8UC3);
    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 480;
    cam.options->framerate=30;
    cam.options->verbose=true;
    cam.startVideo();

    int ch = 0;
    LaneDetectionModule lm({});

    while (ch != 27) {
        if (!cam.getVideoFrame(image, 1000)) {
            std::cerr << "can't get image, timeout happend" << std::endl;
        } else {
            auto result = lm.detect_lane(image);
            if (result.has_value()) {
#ifdef ENABLE_RASPBERRY_DEBUG_IMG
                cv::imshow("Lane Detection Status", result->frame);
#endif // ENABLE_RASPBERRY_DEBUG_IMG
                std::cout << result->direction << std::endl;
                std::cout << result->steer_angle << std::endl;
            }
        }

        ch = cv::waitKey(5);
    }

    cam.stopVideo();
    cv::destroyAllWindows();
}

#endif // ENABLE_RASPBERRY_BUILD

void help(const char* app) {
    std::cerr << app << " [PATH TO IMAGE]" << std::endl;
}

void test_static_image(int argc, char* argv[]) {
    if (argc < 2) {
        help(argv[0]);
        return;
    }

    auto frame = cv::imread(argv[1], cv::IMREAD_UNCHANGED);
    if (frame.empty()) {
        return;
    }

    try {
        LaneDetectionModule lm({});
        auto result = lm.detect_lane(std::move(frame));
        if (result.has_value()) {
            cv::imshow("Lane Detection Status", result->frame);
            std::cout << "direction  : " << result->direction << std::endl;
            std::cout << "steer angle: " << result->steer_angle << std::endl;
        }

        if (cv::waitKey(0) == 27 /*ESC*/) {
            return;
        }
    } catch (...) {
        std::cerr << "can not process image" << std::endl;
    }

    return;
}

int main(int argc, char* argv[]) {
#ifdef ENABLE_RASPBERRY_BUILD
    test_raspberry_camera();
#else
    test_static_image(argc, argv);
#endif // ENABLE_RASPBERRY_BUILD

    return 0;
}
