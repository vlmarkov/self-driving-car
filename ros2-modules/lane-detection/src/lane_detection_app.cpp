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
            auto status = lm.detect_lane(image);
#ifdef ENABLE_RASPBERRY_DEBUG_IMG
            cv::imshow("Lane Detection Status", status.frame);
#endif // ENABLE_RASPBERRY_DEBUG_IMG
            std::cout << status.direction << std::endl;
            std::cout << status.steer_angle << std::endl;
        }

        ch = cv::waitKey(5);
    }

    cam.stopVideo();
    cv::destroyAllWindows();
}

#endif // ENABLE_RASPBERRY_BUILD

void test_static_image(int argc, char* argv[]) {
    auto frame = cv::imread(argv[1], cv::IMREAD_UNCHANGED);
    if (frame.empty()) {
        return;
    }

    try {
        LaneDetectionModule lm({});
        auto status = lm.detect_lane(std::move(frame));

        cv::imshow("Lane Detection Status", status.frame);
        std::cout << status.direction << std::endl;
        std::cout << status.steer_angle << std::endl;

        if (cv::waitKey(3000) >= 0) {
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
